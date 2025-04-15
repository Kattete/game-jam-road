using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.LowLevelPhysics;

public class VehicleControllerScript : MonoBehaviour
{
    [Header("Vehicle References")]
    public Transform centerOfMass; // Empty GameObject to set the center of mass
    public Rigidbody vehicleRigidbody;

    [Header("Wheel Colliders")]
    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;

    [Header("Wheel Meshes")]
    public Transform frontLeftWheelMesh;
    public Transform frontRightWheelMesh;
    public Transform rearLeftWheelMesh;
    public Transform rearRightWheelMesh;

    [Header("Vehicle Properties")]
    public float baseMotorTorque = 500f;
    public float baseMaxSteeringAngle = 30f;
    public float baseBrakeTorque = 3000f;
    public float baseVehicleMass = 1500f;

    [Header("Cone Stack Properties")]
    public ConeCollector coneCollector;
    public Transform coneStackPoint;
    public float maxConeStackHeight = 3f;
    public float baseCenterOfMassHeight = 0.1f; // VERY LOW for stability

    [Header("Cone Physics Impact")]
    public float torqueReductionPerCone = 10f;
    public float steeringReductionPerCone = 0.2f;
    public float massIncreasePerCone = 15f;
    public float comHeightIncreasePerCone = 0.01f; // Very small increment

    [Header("Stability Properties")]
    public float baseWobbleFactor = 0.01f; // Drastically reduced
    public float wobbleIncreasePerCone = 0.005f; // Drastically reduced
    public float wobbleFrequency = 1.0f;
    public float stabilizeButtonCooldown = 5f;

    // Physics Materials
    private PhysicsMaterial lowFrictionMaterial;

    // Private variables
    private float currentStabilizeCooldown = 0f;
    private bool isStabilized = false;
    private float stabilizedTimeRemaining = 0f;
    private float stabilizeDuration = 3f;
    private float wobbleTime = 0f;
    private int lastConeCount = 0;
    private bool hasStartedDriving = false;

    // Input variables
    private float steeringInput;
    private float throttleInput;
    private float brakeInput;
    private bool stabilizeInput;

    // Current effective values
    private float currentMotorTorque;
    private float currentMaxSteeringAngle;
    private float currentWobbleFactor;

    void Awake()
    {
        // Create low friction material for wheels if needed
        lowFrictionMaterial = new PhysicsMaterial("LowFriction");
        lowFrictionMaterial.staticFriction = 0.2f;
        lowFrictionMaterial.dynamicFriction = 0.1f;
        lowFrictionMaterial.frictionCombine = PhysicsMaterialCombine.Minimum;
    }

    void Start()
    {
        // Make sure we have a rigidbody
        if (vehicleRigidbody == null)
            vehicleRigidbody = GetComponent<Rigidbody>();

        // CRITICAL: Initialize all motor torques to ZERO to prevent self-rotation
        frontLeftWheelCollider.motorTorque = 0;
        frontRightWheelCollider.motorTorque = 0;
        rearLeftWheelCollider.motorTorque = 0;
        rearRightWheelCollider.motorTorque = 0;

        // CRITICAL: Make sure brake torque is applied at start to prevent rolling
        frontLeftWheelCollider.brakeTorque = 10000;
        frontRightWheelCollider.brakeTorque = 10000;
        rearLeftWheelCollider.brakeTorque = 10000;
        rearRightWheelCollider.brakeTorque = 10000;

        // Initialize rigidbody properties for extreme stability
        vehicleRigidbody.mass = baseVehicleMass;
        vehicleRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        vehicleRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        vehicleRigidbody.maxAngularVelocity = 1.0f; // Limit max angular velocity

        // Set a VERY low center of mass for extreme stability
        if (centerOfMass != null)
        {
            centerOfMass.localPosition = new Vector3(0, baseCenterOfMassHeight, 0);
            vehicleRigidbody.centerOfMass = centerOfMass.localPosition;
        }
        else
        {
            vehicleRigidbody.centerOfMass = new Vector3(0, baseCenterOfMassHeight, 0);
        }

        // Set initial values
        currentMotorTorque = baseMotorTorque;
        currentMaxSteeringAngle = baseMaxSteeringAngle;
        currentWobbleFactor = baseWobbleFactor;

        // Setup wheel colliders with extremely conservative values
        ConfigureWheelColliders();

        // Freeze rigidbody at start
        StartCoroutine(StabilizeAtStart());

        Debug.Log("Vehicle initialized with center of mass at: " + vehicleRigidbody.centerOfMass);
    }

    IEnumerator StabilizeAtStart()
    {
        // Completely freeze the rigidbody for 1 second at start
        vehicleRigidbody.isKinematic = true;
        yield return new WaitForSeconds(0.5f);

        // Ensure wheels are at rest
        foreach (WheelCollider wheel in new[] {
            frontLeftWheelCollider, frontRightWheelCollider,
            rearLeftWheelCollider, rearRightWheelCollider
        })
        {
            wheel.motorTorque = 0;
            wheel.brakeTorque = 10000; // Full brake
        }

        // Re-enable physics
        vehicleRigidbody.isKinematic = false;

        // Apply full brakes initially - player will need to press accelerator to start moving
        SetBrakeTorque(10000);

        // Additional stability wait
        yield return new WaitForSeconds(0.5f);

        // Log the wheel states to check for issues
        LogWheelStates();

        Debug.Log("Vehicle stabilization complete");
    }

    void LogWheelStates()
    {
        // Debug wheel states
        Debug.Log($"FL wheel: rpm={frontLeftWheelCollider.rpm}, motorTorque={frontLeftWheelCollider.motorTorque}, brakeTorque={frontLeftWheelCollider.brakeTorque}");
        Debug.Log($"FR wheel: rpm={frontRightWheelCollider.rpm}, motorTorque={frontRightWheelCollider.motorTorque}, brakeTorque={frontRightWheelCollider.brakeTorque}");
        Debug.Log($"RL wheel: rpm={rearLeftWheelCollider.rpm}, motorTorque={rearLeftWheelCollider.motorTorque}, brakeTorque={rearLeftWheelCollider.brakeTorque}");
        Debug.Log($"RR wheel: rpm={rearRightWheelCollider.rpm}, motorTorque={rearRightWheelCollider.motorTorque}, brakeTorque={rearRightWheelCollider.brakeTorque}");
    }

    void Update()
    {
        // Get input
        GetInput();

        // Update visual wheels based on wheel colliders
        UpdateWheelMeshes();

        // Handle stabilize cooldown
        HandleStabilizeCooldown();

        // Check if cone count has changed and update physics
        if (coneCollector != null && coneCollector.collectedCones.Count != lastConeCount)
        {
            UpdateVehiclePhysics();
            lastConeCount = coneCollector.collectedCones.Count;
        }

        // Debug controls
        if (Input.GetKeyDown(KeyCode.F3))
        {
            LogVehicleState();
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetVehicle();
        }

        // Special debug: log wheel RPM continuously
        if (Input.GetKeyDown(KeyCode.F4))
        {
            StartCoroutine(MonitorWheelRPM());
        }
    }

    IEnumerator MonitorWheelRPM()
    {
        // Log wheel RPM every 0.5 seconds for 10 seconds
        for (int i = 0; i < 20; i++)
        {
            Debug.Log($"FL RPM: {frontLeftWheelCollider.rpm}, " +
                    $"FR RPM: {frontRightWheelCollider.rpm}, " +
                    $"RL RPM: {rearLeftWheelCollider.rpm}, " +
                    $"RR RPM: {rearRightWheelCollider.rpm}");
            yield return new WaitForSeconds(0.5f);
        }
    }

    void LogVehicleState()
    {
        Debug.Log($"Vehicle State: Pos={transform.position}, Rot={transform.rotation.eulerAngles}");
        Debug.Log($"Rigidbody: Vel={vehicleRigidbody.velocity}, AngVel={vehicleRigidbody.angularVelocity}");
        Debug.Log($"COM: {vehicleRigidbody.centerOfMass}, Mass: {vehicleRigidbody.mass}");
        Debug.Log($"Wheels Grounded: FL={frontLeftWheelCollider.isGrounded}, " +
                 $"FR={frontRightWheelCollider.isGrounded}, " +
                 $"RL={rearLeftWheelCollider.isGrounded}, " +
                 $"RR={rearRightWheelCollider.isGrounded}");
    }

    void FixedUpdate()
    {
        // If the vehicle isn't stable yet, don't apply controls
        if (vehicleRigidbody.isKinematic)
            return;

        // Check if vehicle is severely tipped over and try to auto-correct
        CheckAndCorrectFlipping();

        // Only apply motor torque for first time when throttle is pressed
        if (!hasStartedDriving && Mathf.Abs(throttleInput) > 0.1f)
        {
            // First time acceleration - release brakes
            SetBrakeTorque(0);
            hasStartedDriving = true;
        }

        // Apply vehicle control only if we've started driving
        if (hasStartedDriving)
        {
            ApplyMotorTorque();
            ApplySteering();
            if (brakeInput > 0)
            {
                ApplyBraking();
            }

            // Apply wobble effect if not stabilized
            if (!isStabilized && coneCollector != null && coneCollector.collectedCones.Count > 0)
            {
                ApplyWobble();
            }
        }
    }

    void SetBrakeTorque(float amount)
    {
        frontLeftWheelCollider.brakeTorque = amount;
        frontRightWheelCollider.brakeTorque = amount;
        rearLeftWheelCollider.brakeTorque = amount;
        rearRightWheelCollider.brakeTorque = amount;
    }

    void CheckAndCorrectFlipping()
    {
        // If vehicle is upside down or severely tipped over
        if (Vector3.Dot(transform.up, Vector3.up) < 0.3f)
        {
            // Apply a gentle torque to try to right the vehicle
            vehicleRigidbody.AddTorque(Vector3.Cross(transform.up, Vector3.up) * 500f, ForceMode.Force);

            // If severely flipped and not moving much, apply a stronger correction
            if (Vector3.Dot(transform.up, Vector3.up) < 0.0f && vehicleRigidbody.velocity.magnitude < 1f)
            {
                // Auto-correct by gently rotating back
                Quaternion targetRotation = Quaternion.FromToRotation(transform.up, Vector3.up) * transform.rotation;
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 0.1f);
            }
        }
    }

    void ConfigureWheelColliders()
    {
        WheelCollider[] wheels = new WheelCollider[] {
            frontLeftWheelCollider, frontRightWheelCollider,
            rearLeftWheelCollider, rearRightWheelCollider
        };

        foreach (WheelCollider wheel in wheels)
        {
            // Zero out any existing motor/brake torque
            wheel.motorTorque = 0;
            wheel.brakeTorque = 10000; // Start with full brakes

            // Set wheel mass (realistic values)
            wheel.mass = 20f;

            // Set appropriate suspension distances (smaller for stability)
            wheel.suspensionDistance = 0.1f;

            // Configure suspension spring (stiffer for stability)
            JointSpring spring = new JointSpring();
            spring.spring = 50000f;     // Stiffer spring
            spring.damper = 6000f;      // Higher damping
            spring.targetPosition = 0.5f;
            wheel.suspensionSpring = spring;

            // Configure very conservative friction values
            WheelFrictionCurve fwdFriction = wheel.forwardFriction;
            fwdFriction.stiffness = 0.7f;         // Lower stiffness
            fwdFriction.extremumSlip = 0.4f;      // Lower slip threshold
            fwdFriction.extremumValue = 1.0f;     // Lower friction at extremum
            fwdFriction.asymptoteSlip = 0.8f;     // Lower asymptote slip
            fwdFriction.asymptoteValue = 0.8f;    // Lower friction at asymptote
            wheel.forwardFriction = fwdFriction;

            WheelFrictionCurve sideFriction = wheel.sidewaysFriction;
            sideFriction.stiffness = 0.7f;        // Lower stiffness
            sideFriction.extremumSlip = 0.25f;    // Lower slip threshold
            sideFriction.extremumValue = 1.0f;    // Lower friction at extremum
            sideFriction.asymptoteSlip = 0.5f;    // Lower asymptote slip
            sideFriction.asymptoteValue = 0.8f;   // Lower friction at asymptote
            wheel.sidewaysFriction = sideFriction;
        }
    }

    void GetInput()
    {
        // Get horizontal input for steering
        steeringInput = Input.GetAxis("Horizontal");

        // Get vertical input for acceleration/reverse
        throttleInput = Input.GetAxis("Vertical");

        // Get brake input
        brakeInput = Input.GetKey(KeyCode.Space) ? 1.0f : 0.0f;

        // Get stabilize input (Shift key)
        stabilizeInput = Input.GetKeyDown(KeyCode.LeftShift) || Input.GetKeyDown(KeyCode.RightShift);
    }

    void ApplyMotorTorque()
    {
        // Anti-wheelie check - don't apply torque if front wheels aren't grounded
        bool frontWheelsGrounded = frontLeftWheelCollider.isGrounded && frontRightWheelCollider.isGrounded;

        // If front wheels are in the air and we're accelerating forward, reduce torque
        float torqueMultiplier = 1.0f;
        if (!frontWheelsGrounded && throttleInput > 0.1f)
        {
            torqueMultiplier = 0.3f; // Apply only 30% torque to prevent wheelies
        }

        // Apply very gentle torque to drive wheels
        float torqueToApply = throttleInput * currentMotorTorque * torqueMultiplier;

        // Only apply torque if the vehicle is relatively stable
        if (Vector3.Dot(transform.up, Vector3.up) > 0.7f) // Vehicle is mostly upright
        {
            rearLeftWheelCollider.motorTorque = torqueToApply;
            rearRightWheelCollider.motorTorque = torqueToApply;

            // Make sure front wheels have no motor torque (they're only for steering)
            frontLeftWheelCollider.motorTorque = 0;
            frontRightWheelCollider.motorTorque = 0;
        }
        else
        {
            // Vehicle is tilted - don't apply motor torque to prevent worsening the situation
            rearLeftWheelCollider.motorTorque = 0;
            rearRightWheelCollider.motorTorque = 0;
        }
    }

    void ApplySteering()
    {
        // Calculate current steering angle - apply MORE gradually
        float targetSteeringAngle = steeringInput * currentMaxSteeringAngle;

        // Get current angles
        float currentAngleLeft = frontLeftWheelCollider.steerAngle;
        float currentAngleRight = frontRightWheelCollider.steerAngle;

        // Very gradually interpolate to target angle
        float newAngleLeft = Mathf.Lerp(currentAngleLeft, targetSteeringAngle, Time.deltaTime * 3f);
        float newAngleRight = Mathf.Lerp(currentAngleRight, targetSteeringAngle, Time.deltaTime * 3f);

        // Apply steering to front wheels
        frontLeftWheelCollider.steerAngle = newAngleLeft;
        frontRightWheelCollider.steerAngle = newAngleRight;
    }

    void ApplyBraking()
    {
        // Apply brakes to all wheels when space is pressed
        float brakeTorqueToApply = baseBrakeTorque;

        frontLeftWheelCollider.brakeTorque = brakeTorqueToApply;
        frontRightWheelCollider.brakeTorque = brakeTorqueToApply;
        rearLeftWheelCollider.brakeTorque = brakeTorqueToApply;
        rearRightWheelCollider.brakeTorque = brakeTorqueToApply;
    }

    void UpdateWheelMeshes()
    {
        UpdateWheelMesh(frontLeftWheelCollider, frontLeftWheelMesh);
        UpdateWheelMesh(frontRightWheelCollider, frontRightWheelMesh);
        UpdateWheelMesh(rearLeftWheelCollider, rearLeftWheelMesh);
        UpdateWheelMesh(rearRightWheelCollider, rearRightWheelMesh);
    }

    void UpdateWheelMesh(WheelCollider wheelCollider, Transform wheelMesh)
    {
        if (wheelCollider == null || wheelMesh == null)
            return;

        Vector3 position;
        Quaternion rotation;
        wheelCollider.GetWorldPose(out position, out rotation);

        wheelMesh.position = position;
        wheelMesh.rotation = rotation;
    }

    void UpdateVehiclePhysics()
    {
        if (coneCollector == null)
            return;

        int coneCount = coneCollector.collectedCones.Count;

        // Calculate new physics values based on cone count but with more conservative changes
        currentMotorTorque = Mathf.Max(baseMotorTorque - (coneCount * torqueReductionPerCone), baseMotorTorque * 0.3f);
        currentMaxSteeringAngle = Mathf.Max(baseMaxSteeringAngle - (coneCount * steeringReductionPerCone), baseMaxSteeringAngle * 0.4f);
        currentWobbleFactor = baseWobbleFactor + (coneCount * wobbleIncreasePerCone);

        // Update vehicle mass
        vehicleRigidbody.mass = baseVehicleMass + (coneCount * massIncreasePerCone);

        // Update center of mass height based on cone stack but limit maximum height
        if (centerOfMass != null)
        {
            // Limit the maximum height to prevent extreme instability
            float maxAllowedHeight = 0.5f; // Very low maximum
            float newCOMHeight = Mathf.Min(
                baseCenterOfMassHeight + (coneCount * comHeightIncreasePerCone),
                maxAllowedHeight
            );

            Vector3 newCOM = centerOfMass.localPosition;
            newCOM.y = newCOMHeight;
            centerOfMass.localPosition = newCOM;
            vehicleRigidbody.centerOfMass = centerOfMass.localPosition;
        }

        Debug.Log($"Vehicle updated: Cones={coneCount}, Torque={currentMotorTorque}, " +
                 $"Steering={currentMaxSteeringAngle}, Mass={vehicleRigidbody.mass}, " +
                 $"Wobble={currentWobbleFactor}, COM Height={vehicleRigidbody.centerOfMass.y}");
    }

    void ApplyWobble()
    {
        // Only wobble if certain conditions are met
        if (coneCollector.collectedCones.Count <= 0 || vehicleRigidbody.velocity.magnitude < 2f ||
            !frontLeftWheelCollider.isGrounded || !frontRightWheelCollider.isGrounded)
            return;

        wobbleTime += Time.fixedDeltaTime * wobbleFrequency;

        // Minimal wobble
        float normalizedSpeed = Mathf.Clamp01(vehicleRigidbody.velocity.magnitude / 30f);
        float turningFactor = Mathf.Abs(steeringInput) * 0.3f; // Very low turning factor

        // Extremely gentle wobble force
        float wobbleDirection = Mathf.Sin(wobbleTime) * currentWobbleFactor;

        // Apply almost imperceptible force at first, scaling with cone count
        Vector3 wobbleForce = transform.right * wobbleDirection * normalizedSpeed * turningFactor *
                             (coneCollector.collectedCones.Count * 2f); // Extremely low multiplier

        // Apply force at cone stack's position but with Force mode
        if (coneStackPoint != null)
        {
            vehicleRigidbody.AddForceAtPosition(wobbleForce, coneStackPoint.position, ForceMode.Force);
        }
        else
        {
            Vector3 forcePos = transform.position + Vector3.up * 0.7f;
            vehicleRigidbody.AddForceAtPosition(wobbleForce, forcePos, ForceMode.Force);
        }
    }

    void HandleStabilizeCooldown()
    {
        if (currentStabilizeCooldown > 0)
            currentStabilizeCooldown -= Time.deltaTime;

        if (stabilizeInput && currentStabilizeCooldown <= 0 && !isStabilized)
        {
            ActivateStabilize();
        }

        if (isStabilized)
        {
            stabilizedTimeRemaining -= Time.deltaTime;
            if (stabilizedTimeRemaining <= 0)
            {
                isStabilized = false;
                Debug.Log("Stack stabilization ended");
            }
        }
    }

    void ActivateStabilize()
    {
        isStabilized = true;
        stabilizedTimeRemaining = stabilizeDuration;
        currentStabilizeCooldown = stabilizeButtonCooldown;
        Debug.Log("Stack stabilized for " + stabilizeDuration + " seconds!");
    }

    public void OnConeCollected()
    {
        UpdateVehiclePhysics();
    }

    public float GetStabilizeCooldownPercent()
    {
        return Mathf.Clamp01(currentStabilizeCooldown / stabilizeButtonCooldown);
    }

    public float GetStabilizeActivePercent()
    {
        return isStabilized ? (stabilizedTimeRemaining / stabilizeDuration) : 0f;
    }

    public void ResetVehicle()
    {
        // Reset position slightly above ground
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 100f))
        {
            transform.position = hit.point + Vector3.up * 0.5f;
        }

        // Reset rotation
        transform.rotation = Quaternion.Euler(0, transform.rotation.eulerAngles.y, 0);

        // Reset velocities
        vehicleRigidbody.velocity = Vector3.zero;
        vehicleRigidbody.angularVelocity = Vector3.zero;

        // Reset wheel RPM and torques - critical for stopping self-rotation
        frontLeftWheelCollider.motorTorque = 0;
        frontRightWheelCollider.motorTorque = 0;
        rearLeftWheelCollider.motorTorque = 0;
        rearRightWheelCollider.motorTorque = 0;

        // Apply full brakes
        SetBrakeTorque(10000);

        // Reset driving state
        hasStartedDriving = false;

        Debug.Log("Vehicle has been reset");
    }
}
