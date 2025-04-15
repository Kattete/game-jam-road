using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.UI;
using TMPro;

// Enum to define different cone types
public enum ConeType
{
    Standard,    // Regular traffic cone
    Heavy,       // Construction grade cone, heavier, affects physics more
    Light,       // Small/lightweight cone, affects physics less
    Wobble,      // Special cone that drastically increases instability
    Stabilizer   // Special cone that temporarily reduces wobble effect
}

// Cone class to store individual cone properties
public class Cone : MonoBehaviour
{
    [Header("Cone Properties")]
    public ConeType type = ConeType.Standard;
    public float weight = 15f;
    public float height = 0.5f;
    public float wobbleEffect = 1.0f;

    [Header("Visual Elements")]
    public Material coneMaterial;
    public GameObject warningLight;

    [Header("Special Effect Properties")]
    public float specialEffectDuration = 0f; // For temporary effects like stabilizer
    public float specialEffectStrength = 1.0f; // Multiplier for special effects

    [HideInInspector]
    public bool isCollected = false;
    [HideInInspector]
    public Transform originalParent;

    private void Awake()
    {
        // Store original parent for potential reset
        originalParent = transform.parent;

        // Set cone properties based on its type
        ConfigureConeBasedOnType();
    }

    public void ConfigureConeBasedOnType()
    {
        switch (type)
        {
            case ConeType.Standard:
                weight = 15f;
                height = 0.5f;
                wobbleEffect = 1.0f;
                if (warningLight) warningLight.SetActive(false);
                break;

            case ConeType.Heavy:
                weight = 30f;
                height = 0.7f;
                wobbleEffect = 1.5f;
                transform.localScale = new Vector3(1.2f, 1.2f, 1.2f); // Larger size
                if (coneMaterial) coneMaterial.color = new Color(0.7f, 0.3f, 0.3f); // Darker red
                if (warningLight) warningLight.SetActive(false);
                break;

            case ConeType.Light:
                weight = 5f;
                height = 0.3f;
                wobbleEffect = 0.5f;
                transform.localScale = new Vector3(0.7f, 0.7f, 0.7f); // Smaller size
                if (coneMaterial) coneMaterial.color = new Color(1f, 0.8f, 0.2f); // Yellow
                if (warningLight) warningLight.SetActive(false);
                break;

            case ConeType.Wobble:
                weight = 10f;
                height = 0.5f;
                wobbleEffect = 3.0f;
                specialEffectDuration = 0f; // Permanent effect
                if (coneMaterial) coneMaterial.color = new Color(0.3f, 0.3f, 0.8f); // Blue
                if (warningLight) warningLight.SetActive(true);
                break;

            case ConeType.Stabilizer:
                weight = 20f;
                height = 0.6f;
                wobbleEffect = -2.0f; // Negative value to reduce wobble
                specialEffectDuration = 15f; // Temporary effect (15 seconds)
                specialEffectStrength = 0.5f; // Reduces wobble by half
                if (coneMaterial) coneMaterial.color = new Color(0.3f, 0.8f, 0.3f); // Green
                if (warningLight) warningLight.SetActive(true);
                break;
        }
    }
}

public class ConeCollector : MonoBehaviour
{
    [Header("Collection Properties")]
    public float collectionRadius = 1.5f;
    public Transform stackPoint; // Where cones will be stacked
    public LayerMask coneLayer;
    public int maxCones = 50; // Maximum cones that can be collected

    [Header("Stacking Properties")]
    public float stackSpacing = 0.4f; // Vertical space between cones
    public float stackWobbleAmount = 0.05f; // How much random variation in stacking
    public bool usePhysicsStacking = false; // If true, uses configurable joints for realistic stacking

    [Header("Vehicle References")]
    public VehicleControllerScript vehicleController;
    public Rigidbody vehicleRigidbody;

    [Header("UI References")]
    public TextMeshProUGUI coneCountText;
    public TextMeshProUGUI coneTypeText;
    public Image stackStabilityBar;

    [Header("Effects")]
    public AudioClip collectionSound;
    public ParticleSystem collectionParticle;

    // Lists to track collected cones and active special effects
    [HideInInspector]
    public List<Cone> collectedCones = new List<Cone>();
    private List<Cone> temporaryCones = new List<Cone>(); // Cones with temporary effects

    // Private variables for stacking logic
    private Vector3 nextStackPosition;
    private float currentStackHeight = 0f;
    private bool isStacking = false; // Flag to prevent simultaneous stacking operations

    // Variables to track stack stability
    private float currentStability = 1.0f; // 1.0 = perfectly stable, 0.0 = very unstable
    private float baseTippingThreshold = 10; // Base number of cones before tipping becomes likely

    // Collection cooldown to prevent grabbing too many cones at once
    private float collectionCooldown = 0f;
    private float collectionCooldownDuration = 0.2f;

    private void Start()
    {
        // Initialize cone stack position
        if (stackPoint == null)
        {
            Debug.LogError("Stack point not assigned to ConeCollector!");
            stackPoint = transform;
        }

        nextStackPosition = stackPoint.position;

        // Ensure vehicle controller reference
        if (vehicleController == null)
            vehicleController = GetComponent<VehicleControllerScript>();

        if (vehicleRigidbody == null && vehicleController != null)
            vehicleRigidbody = vehicleController.vehicleRigidbody;

        // Initialize UI if exists
        UpdateUI();
    }

    private void Update()
    {
        // Handle cooldown
        if (collectionCooldown > 0)
            collectionCooldown -= Time.deltaTime;

        // Check for cones to collect
        DetectAndCollectCones();

        // Update temporary effects
        UpdateTemporaryEffects();

        // Update UI elements
        UpdateUI();
    }

    private void FixedUpdate()
    {
        // Calculate current stack stability based on vehicle tilt and velocity
        CalculateStackStability();

        // Check if the stack should collapse
        CheckStackCollapse();
    }

    private void DetectAndCollectCones()
    {
        // Don't try to collect if on cooldown or at max capacity
        if (collectionCooldown > 0 || collectedCones.Count >= maxCones || isStacking)
            return;

        // Use a non-alloc version to avoid garbage
        Collider[] hitColliders = new Collider[5]; // Detect up to 5 cones at once
        int numColliders = Physics.OverlapSphereNonAlloc(
            transform.position,
            collectionRadius,
            hitColliders,
            coneLayer
        );

        for (int i = 0; i < numColliders; i++)
        {
            Cone cone = hitColliders[i].GetComponent<Cone>();

            // Make sure it's a valid cone that hasn't been collected
            if (cone != null && !cone.isCollected)
            {
                StartCoroutine(CollectCone(cone));
                collectionCooldown = collectionCooldownDuration;

                // Only collect one cone per frame to avoid physics issues
                break;
            }
        }
    }

    private IEnumerator CollectCone(Cone cone)
    {
        isStacking = true;

        // Mark as collected
        cone.isCollected = true;

        // Play effects
        if (collectionSound != null)
            AudioSource.PlayClipAtPoint(collectionSound, transform.position);

        if (collectionParticle != null)
            collectionParticle.Play();

        // Add to collection
        collectedCones.Add(cone);

        // If it has a temporary effect, add to that list too
        if (cone.specialEffectDuration > 0)
            temporaryCones.Add(cone);

        // Disable cone's collider
        Collider coneCollider = cone.GetComponent<Collider>();
        if (coneCollider != null)
            coneCollider.enabled = false;

        // Calculate new stack position with slight randomness for realism
        Vector3 randomOffset = new Vector3(
            Random.Range(-stackWobbleAmount, stackWobbleAmount),
            0,
            Random.Range(-stackWobbleAmount, stackWobbleAmount)
        );

        // Calculate vertical position based on previously stacked cones
        Vector3 stackPos = stackPoint.position + Vector3.up * currentStackHeight + randomOffset;

        // Animate cone moving to stack
        float duration = 0.25f;
        Vector3 startPos = cone.transform.position;
        Quaternion startRot = cone.transform.rotation;
        Quaternion targetRot = Quaternion.Euler(0, Random.Range(0, 360), 0);

        for (float t = 0; t < duration; t += Time.deltaTime)
        {
            float normalizedTime = t / duration;
            cone.transform.position = Vector3.Lerp(startPos, stackPos, normalizedTime);
            cone.transform.rotation = Quaternion.Slerp(startRot, targetRot, normalizedTime);
            yield return null;
        }

        // Finalize position
        cone.transform.position = stackPos;
        cone.transform.rotation = targetRot;

        // Attach to vehicle
        cone.transform.SetParent(stackPoint);

        // Update stack height for next cone
        currentStackHeight += cone.height + stackSpacing;

        // If using physics stacking, add a joint
        if (usePhysicsStacking && collectedCones.Count > 1)
        {
            SetupStackingJoint(cone);
        }

        // Update vehicle physics
        UpdateVehiclePhysics();

        // Notify the vehicle controller
        if (vehicleController != null)
            vehicleController.OnConeCollected();

        isStacking = false;
    }

    private void SetupStackingJoint(Cone cone)
    {
        // Add configurable joint for physics-based stacking
        ConfigurableJoint joint = cone.gameObject.AddComponent<ConfigurableJoint>();

        // Get the cone below this one
        Cone coneBelowThis = collectedCones[collectedCones.Count - 2];

        // Connect joint to the cone below
        joint.connectedBody = coneBelowThis.GetComponent<Rigidbody>();

        // Setup joint limits
        joint.xMotion = ConfigurableJointMotion.Limited;
        joint.yMotion = ConfigurableJointMotion.Limited;
        joint.zMotion = ConfigurableJointMotion.Limited;
        joint.angularXMotion = ConfigurableJointMotion.Limited;
        joint.angularYMotion = ConfigurableJointMotion.Limited;
        joint.angularZMotion = ConfigurableJointMotion.Limited;

        // Create soft limits
        SoftJointLimit limit = new SoftJointLimit();
        limit.limit = 0.01f;
        joint.linearLimit = limit;

        SoftJointLimit angularLimit = new SoftJointLimit();
        angularLimit.limit = 5f;
        joint.highAngularXLimit = angularLimit;
        joint.lowAngularXLimit = angularLimit;
        joint.angularYLimit = angularLimit;
        joint.angularZLimit = angularLimit;

        // Add some "bounciness"
        joint.enablePreprocessing = false;

        // Add some damping
        joint.xDrive = SetupJointDrive(500, 50);
        joint.yDrive = SetupJointDrive(500, 50);
        joint.zDrive = SetupJointDrive(500, 50);
        joint.angularXDrive = SetupJointDrive(500, 50);
        joint.angularYZDrive = SetupJointDrive(500, 50);
    }

    private JointDrive SetupJointDrive(float spring, float damper)
    {
        JointDrive drive = new JointDrive();
        drive.positionSpring = spring;
        drive.positionDamper = damper;
        drive.maximumForce = 1000f;
        return drive;
    }

    private void UpdateVehiclePhysics()
    {
        if (vehicleController == null || vehicleRigidbody == null)
            return;

        // Calculate total weight from all cones
        float totalWeight = 0f;
        float totalWobbleEffect = 0f;

        foreach (Cone cone in collectedCones)
        {
            totalWeight += cone.weight;
            totalWobbleEffect += cone.wobbleEffect;
        }

        // Apply physics variables to vehicle controller
        // This is handled in the VehicleController script, we just need to
        // make sure the collected cones list is up to date, which it is

        // We can tweak some parameters based on special cone effects
        float wobbleMultiplier = 1.0f;

        // Apply effects of any temporary special cones
        foreach (Cone cone in temporaryCones)
        {
            if (cone.type == ConeType.Stabilizer && cone.specialEffectDuration > 0)
            {
                // Stabilizer cone reduces wobble
                wobbleMultiplier *= cone.specialEffectStrength;
            }
            else if (cone.type == ConeType.Wobble)
            {
                // Wobble cone increases wobble
                wobbleMultiplier *= (1.0f + (cone.specialEffectStrength * 0.5f));
            }
        }

        // Send this information to the vehicle controller if needed
        // For now, the vehicle controller will just use the cone count
    }

    private void UpdateTemporaryEffects()
    {
        // Handle any cones with temporary effects
        for (int i = temporaryCones.Count - 1; i >= 0; i--)
        {
            Cone cone = temporaryCones[i];

            // Reduce remaining effect duration
            cone.specialEffectDuration -= Time.deltaTime;

            // If effect has expired, remove from temporary list
            if (cone.specialEffectDuration <= 0)
            {
                // For stabilizer cones, they lose their special effect
                if (cone.type == ConeType.Stabilizer)
                {
                    // Convert to standard cone
                    cone.type = ConeType.Standard;
                    cone.ConfigureConeBasedOnType();

                    // Update the vehicle physics since a special cone effect expired
                    UpdateVehiclePhysics();
                }

                temporaryCones.RemoveAt(i);
            }
        }
    }

    private void CalculateStackStability()
    {
        if (collectedCones.Count == 0 || vehicleRigidbody == null)
        {
            currentStability = 1.0f;
            return;
        }

        // Calculate stability based on several factors:
        // 1. Vehicle's tilt angle
        // 2. Vehicle's angular velocity
        // 3. Number of cones relative to tipping threshold
        // 4. Vehicle's speed

        // Get vehicle's up vector relative to world up
        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
        float normalizedTilt = Mathf.Clamp01(tiltAngle / 45f); // 45 degrees is very unstable

        // Get angular velocity (how fast the vehicle is rotating)
        float angularSpeed = vehicleRigidbody.angularVelocity.magnitude;
        float normalizedAngular = Mathf.Clamp01(angularSpeed / 2f); // 2 rad/s is very unstable

        // Get current speed and normalize
        float speed = vehicleRigidbody.linearVelocity.magnitude;
        float normalizedSpeed = Mathf.Clamp01(speed / 15f); // 15 units/s is fast

        // Calculate cone stack factor
        float stackFactor = Mathf.Clamp01((float)collectedCones.Count / baseTippingThreshold);

        // Calculate stability (1.0 = perfectly stable, 0.0 = about to tip)
        currentStability = 1.0f - (
            (normalizedTilt * 0.4f) +
            (normalizedAngular * 0.3f) +
            (stackFactor * 0.2f) +
            (normalizedSpeed * stackFactor * 0.1f)
        );

        // Clamp result
        currentStability = Mathf.Clamp01(currentStability);
    }

    private void CheckStackCollapse()
    {
        // If stability is very low and we have cones, we might lose some
        if (currentStability < 0.2f && collectedCones.Count > 3 && !isStacking)
        {
            // More likely to collapse with lower stability
            float collapseChance = (0.2f - currentStability) * 5f; // 0% at 0.2 stability, 100% at 0 stability

            if (Random.value < collapseChance * Time.fixedDeltaTime)
            {
                // Determine how many cones to drop (more at lower stability)
                int conesToDrop = Mathf.CeilToInt(collectedCones.Count * (1f - currentStability) * 0.5f);
                conesToDrop = Mathf.Clamp(conesToDrop, 1, collectedCones.Count - 1);

                // Drop them
                StartCoroutine(CollapseStack(conesToDrop));
            }
        }
    }

    private IEnumerator CollapseStack(int conesToDrop)
    {
        isStacking = true;

        // Drop cones from the top of the stack
        for (int i = 0; i < conesToDrop; i++)
        {
            if (collectedCones.Count == 0)
                break;

            // Get the topmost cone
            Cone coneToDetach = collectedCones[collectedCones.Count - 1];

            // Remove from lists
            collectedCones.Remove(coneToDetach);
            if (temporaryCones.Contains(coneToDetach))
                temporaryCones.Remove(coneToDetach);

            // Detach from stack
            coneToDetach.isCollected = false;
            coneToDetach.transform.SetParent(null);

            // Re-enable collider
            Collider coneCollider = coneToDetach.GetComponent<Collider>();
            if (coneCollider != null)
                coneCollider.enabled = true;

            // Add rigidbody if it doesn't have one
            Rigidbody coneRb = coneToDetach.GetComponent<Rigidbody>();
            if (coneRb == null)
                coneRb = coneToDetach.gameObject.AddComponent<Rigidbody>();

            // Add force to scatter the cone
            Vector3 scatterDirection = new Vector3(
                Random.Range(-1f, 1f),
                Random.Range(0.5f, 1f),
                Random.Range(-1f, 1f)
            ).normalized;

            coneRb.AddForce(scatterDirection * Random.Range(5f, 10f), ForceMode.Impulse);
            coneRb.AddTorque(Random.insideUnitSphere * 5f, ForceMode.Impulse);

            // Recalculate stack height
            UpdateStackHeight();

            // Update vehicle physics
            UpdateVehiclePhysics();

            // Briefly pause between drops
            yield return new WaitForSeconds(0.05f);
        }

        // Notify the vehicle controller
        if (vehicleController != null)
            vehicleController.OnConeCollected();

        isStacking = false;
    }

    private void UpdateStackHeight()
    {
        // Recalculate total stack height based on remaining cones
        currentStackHeight = 0f;
        foreach (Cone cone in collectedCones)
        {
            currentStackHeight += cone.height + stackSpacing;
        }
    }

    private void UpdateUI()
    {
        if (coneCountText != null)
            coneCountText.text = $"Cones: {collectedCones.Count}";

        if (stackStabilityBar != null)
            stackStabilityBar.fillAmount = currentStability;

        if (coneTypeText != null && collectedCones.Count > 0)
        {
            // Count cone types
            int standard = 0, heavy = 0, light = 0, wobble = 0, stabilizer = 0;

            foreach (Cone cone in collectedCones)
            {
                switch (cone.type)
                {
                    case ConeType.Standard: standard++; break;
                    case ConeType.Heavy: heavy++; break;
                    case ConeType.Light: light++; break;
                    case ConeType.Wobble: wobble++; break;
                    case ConeType.Stabilizer: stabilizer++; break;
                }
            }

            coneTypeText.text = $"Standard: {standard} | Heavy: {heavy} | Light: {light} | Wobble: {wobble} | Stabilizer: {stabilizer}";
        }
    }

    // Method to get total weight of cone stack (for other scripts)
    public float GetTotalConeWeight()
    {
        float totalWeight = 0f;
        foreach (Cone cone in collectedCones)
        {
            totalWeight += cone.weight;
        }
        return totalWeight;
    }

    // Method to get total wobble effect (for other scripts)
    public float GetTotalWobbleEffect()
    {
        float totalWobble = 0f;
        float wobbleMultiplier = 1.0f;

        // Calculate base wobble from all cones
        foreach (Cone cone in collectedCones)
        {
            totalWobble += cone.wobbleEffect;
        }

        // Apply special modifiers from temporary cones
        foreach (Cone cone in temporaryCones)
        {
            if (cone.type == ConeType.Stabilizer && cone.specialEffectDuration > 0)
            {
                wobbleMultiplier *= cone.specialEffectStrength;
            }
        }

        return totalWobble * wobbleMultiplier;
    }

    // Utility method to get cone counts by type (for scoring, etc.)
    public Dictionary<ConeType, int> GetConeCountsByType()
    {
        Dictionary<ConeType, int> counts = new Dictionary<ConeType, int>();

        foreach (ConeType type in System.Enum.GetValues(typeof(ConeType)))
        {
            counts[type] = 0;
        }

        foreach (Cone cone in collectedCones)
        {
            counts[cone.type]++;
        }

        return counts;
    }

    // Debug gizmos to show collection radius
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, collectionRadius);

        if (stackPoint != null)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(stackPoint.position, 0.2f);
            Gizmos.DrawLine(transform.position, stackPoint.position);
        }
    }
}