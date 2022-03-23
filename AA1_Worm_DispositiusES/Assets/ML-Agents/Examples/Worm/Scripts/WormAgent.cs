using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class WormAgent : Agent
{
    public bool isTrialWorm = false;

    const float m_MaxWalkingSpeed = 10; //The max walking speed

    [Header("Target Prefabs")] public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")] public List<Transform> bodySegments;
    [SerializeField]int actualBodyParts = 4;
    //public Transform bodySegment0;
    //public Transform bodySegment1;
    //public Transform bodySegment2;
    //public Transform bodySegment3;

    bool touchedTarget = false;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    private Vector3 m_StartingPos; //starting position of the agent

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position); //spawn target

        m_StartingPos = bodySegments[0].position;
        //m_StartingPos = bodySegment0.position;
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        UpdateOrientationObjects();
        //actualBodyParts = Random.Range(4, bodySegments.Count);
        //Setup each body part
        for (int i = 0; i < bodySegments.Count; i++)
        {
            if (i == actualBodyParts - 1)
            {
                // Do nothing
            }
            else if (i < actualBodyParts)
                m_JdController.SetupBodyPart(bodySegments[i]);
            else
            {
                //bodySegments[i].gameObject.SetActive(false);
                bodySegments[i].gameObject.GetComponentInChildren<MeshRenderer>().enabled = false;
                bodySegments[i].transform.GetChild(0).GetChild(0).GetComponent<MeshRenderer>().enabled = false;
                //bodySegments[i].transform.GetChild(0).GetComponentInChildren<MeshRenderer>().enabled = false;
                //bodySegments[i].gameObject.GetComponentInChildren<CapsuleCollider>().enabled = false;
                bodySegments[i].gameObject.GetComponentInChildren<Rigidbody>().useGravity = false;
            }
        }

        //if(rnd < bodySegments.Length - 1 )
        //{
        //    bodySegments[rnd + 1].gameObject.SetActive(false);
        //}
        //m_JdController.SetupBodyPart(bodySegment0);
        //m_JdController.SetupBodyPart(bodySegment1);
        //m_JdController.SetupBodyPart(bodySegment2);
        //m_JdController.SetupBodyPart(bodySegment3);
    }
    

    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        //bodySegment0.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);
        bodySegments[0].rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0); // Whether the bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));


        if (bp.rb.transform != bodySegments[0])
        {
            //Get position relative to hips in the context of our orientation cube's space
            //sensor.AddObservation(
            //    m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - bodySegment0.position));
            sensor.AddObservation(
                m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - bodySegments[0].position));
            sensor.AddObservation(bp.rb.transform.localRotation);
        }

        if (bp.joint)
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        RaycastHit hit;
        float maxDist = 10;
        //if (Physics.Raycast(bodySegment0.position, Vector3.down, out hit, maxDist)) // Mide distancia suelo
        //{
        //    sensor.AddObservation(hit.distance / maxDist);
        //}
        if (Physics.Raycast(bodySegments[0].position, Vector3.down, out hit, maxDist)) // Mide distancia suelo
        {
            sensor.AddObservation(hit.distance / maxDist);
        }
        else
            sensor.AddObservation(1);

        var cubeForward = m_OrientationCube.transform.forward;
        var velGoal = cubeForward * m_MaxWalkingSpeed;
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        //sensor.AddObservation(Quaternion.Angle(m_OrientationCube.transform.rotation,
        //                          m_JdController.bodyPartsDict[bodySegment0].rb.rotation) / 180);
        //sensor.AddObservation(Quaternion.FromToRotation(bodySegment0.forward, cubeForward));
        sensor.AddObservation(Quaternion.Angle(m_OrientationCube.transform.rotation,
                                  m_JdController.bodyPartsDict[bodySegments[0]].rb.rotation) / 180);
        sensor.AddObservation(Quaternion.FromToRotation(bodySegments[0].forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        //Add number of current pieces
        sensor.AddObservation(actualBodyParts);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
        touchedTarget = true;

    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;
        // Pick a new target joint rotation
        foreach(Transform segment in bodySegments)
        {
            if (segment.gameObject.GetComponent<ConfigurableJoint>() != null && i < (actualBodyParts - 2) * 2)
            {
                //if (segment.gameObject.activeInHierarchy)
                //{
                    bpDict[segment].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
                //}
            }
        }
        i = (actualBodyParts - 2) * 2;
        foreach (Transform segment in bodySegments)
        {
            if (segment.gameObject.GetComponent<ConfigurableJoint>() != null && i < bodySegments.Count)
            {
                //if (segment.gameObject.activeInHierarchy)
                //{
                    bpDict[segment].SetJointStrength(continuousActions[++i]);
                //}
            }
        }
        //bpDict[bodySegment0].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        //bpDict[bodySegment1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        //bpDict[bodySegment2].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);


        //// Update joint strength
        //bpDict[bodySegment0].SetJointStrength(continuousActions[++i]);
        //bpDict[bodySegment1].SetJointStrength(continuousActions[++i]);
        //bpDict[bodySegment2].SetJointStrength(continuousActions[++i]);

        //Reset if Worm fell through floor;
        //if (bodySegment0.position.y < m_StartingPos.y - 2)
        //{
        //    EndEpisode();
        //}
        if (bodySegments[0].position.y < m_StartingPos.y - 2)
        {
           EndEpisode();
        }
    }

    void FixedUpdate()
    {
        if (touchedTarget)
        {
            touchedTarget = false;
            if (actualBodyParts < bodySegments.Count)
            {
                //bodySegments[actualBodyParts].gameObject.SetActive(true);
                bodySegments[actualBodyParts].gameObject.GetComponentInChildren<MeshRenderer>().enabled = true;
                bodySegments[actualBodyParts].transform.GetChild(0).GetChild(0).GetComponent<MeshRenderer>().enabled = true;
                //bodySegments[actualBodyParts].gameObject.GetComponentInChildren<CapsuleCollider>().enabled = true;
                bodySegments[actualBodyParts].gameObject.GetComponentInChildren<Rigidbody>().useGravity = true;
                //bodySegments[actualBodyParts - 1].gameObject.AddComponent<ConfigurableJoint>();
                //GameObject jointGO = bodySegments[actualBodyParts - 1].gameObject;
                //ConfigurableJoint prevJoint = bodySegments[actualBodyParts - 2].gameObject.GetComponent<ConfigurableJoint>();
                //jointGO.GetComponent<ConfigurableJoint>().xMotion = prevJoint.xMotion;
                //jointGO.GetComponent<ConfigurableJoint>().yMotion = prevJoint.yMotion;
                //jointGO.GetComponent<ConfigurableJoint>().zMotion = prevJoint.zMotion;
                //jointGO.GetComponent<ConfigurableJoint>().angularXMotion = prevJoint.angularXMotion;
                //jointGO.GetComponent<ConfigurableJoint>().angularYMotion = prevJoint.angularYMotion;
                //jointGO.GetComponent<ConfigurableJoint>().angularZMotion = prevJoint.angularZMotion;


                //jointGO.GetComponent<ConfigurableJoint>().connectedBody = bodySegments[actualBodyParts].gameObject.GetComponent<Rigidbody>();


                //joint.connectedBody = bodySegments[actualBodyParts].gameObject.GetComponent<Rigidbody>();
                //Transform currBodySegment = bodySegments[actualBodyParts];
                //Physics.IgnoreCollision(bodySegments[actualBodyParts].GetComponent<Collider>(), bodySegments[actualBodyParts - 1].GetComponent<Collider>());
                //bodySegments[actualBodyParts].transform.position = bodySegments[actualBodyParts - 1].transform.position;
                m_JdController.SetupBodyPart(bodySegments[actualBodyParts - 1]);
                actualBodyParts++;
            }
        }

        UpdateOrientationObjects();

        //var velReward =
        //    GetMatchingVelocityReward(m_OrientationCube.transform.forward * m_MaxWalkingSpeed,
        //        m_JdController.bodyPartsDict[bodySegment0].rb.velocity);
        var velReward = 
            GetMatchingVelocityReward(m_OrientationCube.transform.forward * m_MaxWalkingSpeed,
                m_JdController.bodyPartsDict[bodySegments[0]].rb.velocity);

        //Angle of the rotation delta between cube and body.
        //This will range from (0, 180)
        //var rotAngle = Quaternion.Angle(m_OrientationCube.transform.rotation,
        //    m_JdController.bodyPartsDict[bodySegment0].rb.rotation);
        var rotAngle = Quaternion.Angle(m_OrientationCube.transform.rotation,
            m_JdController.bodyPartsDict[bodySegments[0]].rb.rotation);

        //The reward for facing the target
        var facingRew = 0f;
        //If we are within 30 degrees of facing the target
        if (rotAngle < 30)
        {
            //Set normalized facingReward
            //Facing the target perfectly yields a reward of 1
            facingRew = 1 - (rotAngle / 180);
        }

        //Add the product of these two rewards
        AddReward(velReward * facingRew);
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, m_MaxWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / m_MaxWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(bodySegments[0], m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    private void OnCollisionEnter(Collision col)
    {
        if (col.gameObject.CompareTag("bodySeg") || col.gameObject.CompareTag("ceiling"))
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }
}
