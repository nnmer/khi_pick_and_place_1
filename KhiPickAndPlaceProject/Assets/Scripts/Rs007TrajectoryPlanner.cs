using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
// using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.Rs007Control;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;


using RsJ1Msg = RosMessageTypes.Rs007Control.Rs007Joints1Msg;
using RsJ6Msg = RosMessageTypes.Rs007Control.Rs007Joints6Msg;
// using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
// using MmTrayMsg = RosMessageTypes.Rs007Control.MagneMotionTrayMsg;

public enum GripperType {  None, Claw, Vacuum };
public enum GripTrigger {  OnCollision, OnCommand };
public enum MovementStyle {  PhysicsForces, MagicMovement };

public class Rs007TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "rs007_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_RobotModel;
    public GameObject RobotModel { get => m_RobotModel; set => m_RobotModel = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }


    public float posePauseSecs = 2;

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    Vector3 m_PlacePoseOffset = Vector3.up * 0.15f;

    // offsets to make it more flexible for position changing
    Vector3 m_robotOffset = Vector3.zero;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper = null;
    ArticulationBody m_RightGripper = null;


    GameObject m_VacGripper = null;
    Collider m_TargCollider = null;

    GripperType gripperType;
    GripTrigger gripTrigger;
    MovementStyle movementStyle;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        movementStyle = MovementStyle.MagicMovement;
        movementStyle = MovementStyle.PhysicsForces;

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        if (m_RobotModel != null)
        {
            m_robotOffset = m_RobotModel.transform.position;
            Debug.Log($"Robot name:{m_RobotModel.name}");
            Debug.Log($"Robot offset:{m_robotOffset:f3}");
        }

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += Rs007SourceDestinationPublisher.LinkNames[i];
            var xform = m_RobotModel.transform.Find(linkName);
            m_JointArticulationBodies[i] = xform.GetComponent<ArticulationBody>();
            if (movementStyle == MovementStyle.MagicMovement)
            {
                //var rigid = xform.GetComponent<RigidBody>();
                //   m_JointArticulationBodies[i].enabled = false;
            }
        }

        // Find left and right fingers
        var rightGripperName = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripperName = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";
        var vacGripperName = linkName + "/tool_link/GRIPPER v5_1";
        var rgrip = m_RobotModel.transform.Find(rightGripperName);
        var lgrip = m_RobotModel.transform.Find(leftGripperName);
        var vgrip = m_RobotModel.transform.Find(vacGripperName);
        if (vgrip==null)
        {
            vacGripperName = linkName + "/tool_link/gripper_base/Visuals/unnamed/RS007_Gripper_C_u";
            vgrip = m_RobotModel.transform.Find(vacGripperName);
        }


        if (vgrip != null)
        {
            Debug.Log("Vacuum Gripper found");
            gripperType = GripperType.Vacuum;
            m_VacGripper = vgrip.gameObject;
            //m_PickPoseOffset = Vector3.up * 0.1865f;
            m_PlacePoseOffset = Vector3.up * 0.22f;
            m_PickPoseOffset = Vector3.up * 0.22f;
            gripTrigger = GripTrigger.OnCommand;
            if (Target != null)
            {
                m_TargCollider = Target.GetComponent<Collider>();
            }
            else
            {
                Debug.LogError("Target is null for Vacumm Gripper");
            }
        }
        else if (rgrip != null && lgrip != null)
        {
            Debug.Log("Claw Gripper found");
            gripperType = GripperType.Claw;
            m_RightGripper = rgrip.GetComponent<ArticulationBody>();
            m_LeftGripper = lgrip.GetComponent<ArticulationBody>();
            m_PickPoseOffset = Vector3.up * 0.1f;
            m_PlacePoseOffset = Vector3.up * 0.15f;
            gripTrigger = GripTrigger.OnCommand;
        }
        else
        {
            Debug.LogError("No Gripper found");
            gripperType = GripperType.None;
        }
        ROSConnection.GetOrCreateInstance().Subscribe<RsJ1Msg>("Rs007Joints1", Rs007J1Change);
        ROSConnection.GetOrCreateInstance().Subscribe<RsJ6Msg>("Rs007Joints6", Rs007J6Change);


        Debug.Log("Finished Rs007TrajectoryPlanner Start");
        //OpenGripper();
    }

    void PositionJoint(int idx, float joint)
    {
        Debug.Log($"   PositionJoint idx:{idx} to {joint:f1} degrees");
        if (0 <= idx && idx <= 5)
        {
            var joint1XDrive = m_JointArticulationBodies[idx].xDrive;
            joint1XDrive.target = joint;
            m_JointArticulationBodies[idx].xDrive = joint1XDrive;
        }
    }

    void Rs007J1Change(RsJ1Msg j1msg)
    {
        Debug.Log($"RsJ1Msg:{j1msg.ToString()}");
        var idx = j1msg.idx;
        var joint = (float) j1msg.joint;
        PositionJoint(idx, joint);
    }

    void Rs007J6Change(RsJ6Msg j6msg)
    {
        Debug.Log($"RsJ6Msg:{j6msg.ToString()}");
        for(int i=0; i<6; i++)
        {
            PositionJoint( i, (float) j6msg.joints[i] );
        }

    }



    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        switch (gripperType)
        {
            case GripperType.Claw:
                {
                    var leftDrive = m_LeftGripper.xDrive;
                    var rightDrive = m_RightGripper.xDrive;

                    leftDrive.target = -0.01f;
                    rightDrive.target = 0.01f;

                    m_LeftGripper.xDrive = leftDrive;
                    m_RightGripper.xDrive = rightDrive;
                    break;
                }
            case GripperType.Vacuum:
                {
                    m_TargCollider.transform.parent = m_VacGripper.transform;
                    break;
                }
        }
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        switch (gripperType)
        {
            case GripperType.Claw:
                {
                    var leftDrive = m_LeftGripper.xDrive;
                    var rightDrive = m_RightGripper.xDrive;

                    leftDrive.target = 0.01f;
                    rightDrive.target = -0.01f;

                    m_LeftGripper.xDrive = leftDrive;
                    m_RightGripper.xDrive = rightDrive;
                    break;
                }
            case GripperType.Vacuum:
                {
                    m_TargCollider.transform.parent = null;
                    break;
                }
        }
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    // NiryoMoveitJoints CurrentJointConfig()
    Rs007MoveitJointsMsg CurrentJointConfig()
    {
        // var joints = new NiryoMoveitJointsMsg();
        var joints = new Rs007MoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        //var off = new Vector3(0, -m_robotFloorHeight, 0);
        var roboff = -m_robotOffset;

        // Pick Pose
        var pt1 = (m_Target.transform.position + m_PickPoseOffset + roboff).To<FLU>();
        var or1 = Quaternion.Euler(180, m_Target.transform.eulerAngles.y, 0).To<FLU>();
        request.pick_pose = new PoseMsg
        {
            position = pt1,

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            // orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
            orientation = or1
        };
        var msg0 = $"m_PickPoseOffset:{m_PickPoseOffset:f3}";
        Debug.Log(msg0);
        var msg00 = $"roboff:{roboff:f3}";
        Debug.Log(msg00);
        var msg1 = $"PickPose (FLU) position:{pt1:f3} orientation:{or1:f3}";
        Debug.Log(msg1);

        // Place Pose
        var pt2 = (m_TargetPlacement.transform.position + m_PlacePoseOffset + roboff).To<FLU>();
        var or2 = Quaternion.Euler(180, 0, 0).To<FLU>();
        request.place_pose = new PoseMsg
        {
            position = pt2,
            orientation = or2
        };
        var msg2 = $"PlacePose (FLU) position:{pt2:f3} orientation:{or2:f3}";
        Debug.Log(msg2);

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }
    string[] poses = { "PreGrasp", "Grasp", "Pickup", "Place" };

    void RealizeJoints(float[] joints)
    {
        int i = 0;
        var q = Quaternion.identity;
        foreach (var joint in joints)
        {
            var joint1XDrive = m_JointArticulationBodies[i].xDrive;
            joint1XDrive.target = joint;
            m_JointArticulationBodies[i].xDrive = joint1XDrive;
            switch (i)
            {
                case 0:  // link1
                    q = Quaternion.Euler(0, joint, 0);
                    break;
                case 1:  // link2
                    q = Quaternion.Euler(-90 + joint, 90, -90);
                    break;
                case 2:  // link3
                    q = Quaternion.Euler(0, joint, 0);
                    break;
                case 3:  // link4
                    q = Quaternion.Euler(joint - 200, -90, -90);
                    break;
                case 4:  // link5
                    q = Quaternion.Euler(-90 - joint, -90, 90);
                    break;
                case 5:  // link6
                    q = Quaternion.Euler(90 + joint, 90, 90);
                    break;
            }
            m_JointArticulationBodies[i].transform.rotation = q;
            i++;
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                int trajidx = 0;
                // For every robot pose in trajectory plan
                int lsttrajidx = response.trajectories[poseIndex].joint_trajectory.points.Length - 1;
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {

                    var jointPositions = t.positions;
                    var resultRad = jointPositions.Select(r => (float)r).ToArray();
                    //                    var result = jointPositions.Select(r => (float)r).ToArray();
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    if (trajidx == 0 || trajidx == lsttrajidx)
                    {
                        var posword = poses[poseIndex];
                        var msg = $"pose {posword} {poseIndex}:{poseIndex}-{trajidx} start-joints - ";
                        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                        {
                            msg += $"{resultRad[joint]:f4}   ";
                        }
                        msg += "     degrees:";
                        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                        {
                            msg += $"{result[joint]:f1}   ";
                        }

                        Debug.Log(msg);
                    }

                    switch (movementStyle)
                    {
                        case MovementStyle.PhysicsForces:
                            {
                                // Set the joint values for every joint
                                for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                                {
                                    var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                                    joint1XDrive.target = result[joint];
                                    m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                                }
                                break;
                            }
                        case MovementStyle.MagicMovement:
                            {
                                RealizeJoints(result);
                                break;
                            }
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                    trajidx++;
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                    Debug.Log("CloseGripper");
                }

                // Wait for the robot to achieve the final pose from joint assignment
                Debug.Log($"Sleeping {posePauseSecs} secs");
                yield return new WaitForSeconds(k_PoseAssignmentWait + posePauseSecs);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
            yield return new WaitForSeconds(k_PoseAssignmentWait + posePauseSecs);
            Debug.Log($"Sleeping {posePauseSecs} secs");
            Debug.Log("OpenGripper");
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}

