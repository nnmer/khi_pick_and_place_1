using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KhiDemo
{

    public enum RobotCmdE { MoveTo }
    public class RobotSingleCommand
    {
        public MagneMotion magmo;
        public RobotCmdE cmd;
        public float duration;
        public float[] targetPos;

        public RobotSingleCommand(MagneMotion magmo,RobotCmdE cmd, float duration, float [] targetPos )
        {
            this.magmo = magmo;
            this.cmd = cmd;
            this.duration = duration;
            this.targetPos = targetPos;
        }
    }
    public class RobotCommand
    {
        public MagneMotion magmo;
        MmRobot robot;
        List<RobotSingleCommand> commands;
        public float duration;
        public float[] startPos;
        public RobotCommand(MagneMotion magmo,MmRobot robot)
        {
            this.magmo = magmo;
            this.robot = robot;
            commands = new List<RobotSingleCommand>();
            duration = 0;
            startPos = robot.GetRobotPos();
        }
        public void Add(RobotSingleCommand cmd)
        {
            commands.Add(cmd);
        }
    }

    public enum RobotPose { 
        zero, deg10, rest, key, ecartup,ecartdn, fcartup, fcartdn, 
        key00up, key00dn, key01up, key01dn, key02up, key02dn, key03up, key03dn,
        key10up, key10dn, key11up, key11dn, key12up, key12dn, key13up, key13dn,
        key20up, key20dn, key21up, key21dn, key22up, key22dn, key23up, key23dn,
    }

    public class MmRobot : MonoBehaviour
    {
        public bool loadState;
        public MagneMotion magmo;
        public Transform vgriptrans;
        public MmBox box;

        public RobotPose currentRobotPose;

        void Start()
        {
            magmo = FindObjectOfType<MagneMotion>();
            if (magmo==null)
            {
                Debug.LogError($"MmRobot counld not find object of type MagneMotion");
            }
            var vacGripperName = "world/base_link/link1/link2/link3/link4/link5/link6/tool_link/gripper_base/Visuals/unnamed/RS007_Gripper_C_u";
            vgriptrans = transform.Find(vacGripperName);
            InitializePoses();
        }

        Dictionary<RobotPose, float []> poses;
        Dictionary<(int,int),(RobotPose,RobotPose)> kez;

        public void p1(RobotPose pose,(double a1, double a2, double a3, double a4, double a5, double a6) ptd)
        {
            var poseTuple = new float [6] { (float)ptd.a1, (float)ptd.a2, (float)ptd.a3, (float)ptd.a4, (float)ptd.a5, (float)ptd.a6 };
            poses[pose] = poseTuple;
        }
        public void k1((int,int) key,(RobotPose,RobotPose) poses)
        {
            kez[key] = poses;
        }
        public (RobotPose rpup,RobotPose rpdn) GetPoses((int,int) key)
        {
            return kez[key];
        }
        public void InitializePoses()
        {
            poses = new Dictionary<RobotPose, float[]>();
            p1(RobotPose.zero, (0, 0, 0, 0, 0, 0));
            p1(RobotPose.deg10, (10, 10, 10, 10, 10, 10));
            p1(RobotPose.rest, (-16.805, 16.073, -100.892, 0, -63.021, 106.779));

            
            p1(RobotPose.fcartup, (-26.76, 32.812, -74.172, 0, -73.061, 26.755));
            p1(RobotPose.fcartdn, (-26.749, 40.511, -80.809, 0, -58.682, 26.75));

            p1(RobotPose.ecartup, (-50.35, 49.744, -46.295, 0, -83.959, 50.347));
            p1(RobotPose.ecartdn, (-50.351, 55.206, -54.692, 0, -70.107, 50.352));

            p1(RobotPose.key00up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key00dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key01up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key01dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key02up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key02dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key03up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key03dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));

            p1(RobotPose.key10up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key10dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key11up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key11dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key12up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key12dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key13up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key13dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));

            p1(RobotPose.key20up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key20dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key21up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key21dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key22up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key22dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));
            p1(RobotPose.key23up, (-14.864, 26.011, -87.161, 0, -66.826, 102.537));
            p1(RobotPose.key23dn, (-14.48, 28.642, -89.821, 0, -61.519, 104.49));

            kez = new Dictionary<(int, int), (RobotPose,RobotPose)>();
            k1((0, 0), (RobotPose.key00up, RobotPose.key00dn));
            k1((0, 1), (RobotPose.key01up, RobotPose.key01dn));
            k1((0, 2), (RobotPose.key02up, RobotPose.key02dn));
            k1((0, 3), (RobotPose.key03up, RobotPose.key03dn));

            k1((1, 0), (RobotPose.key10up, RobotPose.key10dn));
            k1((1, 1), (RobotPose.key11up, RobotPose.key11dn));
            k1((1, 2), (RobotPose.key12up, RobotPose.key12dn));
            k1((1, 3), (RobotPose.key13up, RobotPose.key13dn));

            k1((2, 0), (RobotPose.key20up, RobotPose.key20dn));
            k1((2, 1), (RobotPose.key21up, RobotPose.key21dn));
            k1((2, 2), (RobotPose.key22up, RobotPose.key22dn));
            k1((2, 3), (RobotPose.key23up, RobotPose.key23dn));
        }

        public void AddBoxToRobot()
        {
            if (vgriptrans == null)
            {
                Debug.LogError("AddBoxToRobot - Robot is null");
                return;
            }
            var lbox = MmBox.ConstructBox(magmo, "rbx", BoxStatus.onRobot);
            AttachBoxToRobot(lbox);
            ActivateRobBox(false);
        }
        public void AttachBoxToRobot(MmBox box)
        {
            if (vgriptrans == null)
            {
                Debug.LogError("AttachBoxToRobot - Robot is null");
                return;
            }
            if (box==null)
            {
                Debug.LogError("AttachBoxToRobot - Box is null");
                return;
            }
            this.box = box;
            box.transform.parent = null;
            box.transform.localRotation = Quaternion.Euler(90, 0, 0);
            box.transform.localPosition = new Vector3(0, -0.14f, 0);
            box.transform.SetParent(vgriptrans, worldPositionStays: false);
            loadState = true;
            box.SetBoxStatus(BoxStatus.onRobot);
        }
        public MmBox DetachhBoxFromRobot()
        {
            var rv = box;
            box = null;
            loadState = false;
            return rv;
        }


        public void InitRobotBoxState(bool startLoadState)
        {
            if (magmo.mmctrl.mmBoxMode== MmBoxMode.Fake)
            {
                AddBoxToRobot();
                ActivateRobBox(startLoadState);
            }
            else if (magmo.mmctrl.mmBoxMode == MmBoxMode.Real)
            {
                if( startLoadState)
                {
                    AddBoxToRobot();
                }
            }
        }
        public bool ActivateRobBox(bool newstat)
        {
            var rv = false;
            if (box != null)
            {
                rv = box.gameObject.activeSelf;
                box.gameObject.SetActive(newstat);
                loadState = newstat;
            }
            return rv;
        }

        public void RealiseRobotPose(RobotPose pose)
        {
            if (!poses.ContainsKey(pose))
            {
                Debug.LogError($"MmRobot.AssumpPose pose {pose} not assigned");
                return;
            }
            var angles = poses[pose];
            var planner = magmo.planner;
            for (int i = 0; i < angles.Length; i++)
            {
                planner.PositionJoint(i, angles[i]);
            }
        }

        public float [] GetRobotPos()
        {
            var far = new List<float>();
            var planner = magmo.planner;
            for (int i = 0; i < 6; i++)
            {
                far.Add(planner.GetJointPosition(i));
            }
            return far.ToArray();
        }

        void DoRobotPose()
        {
            if (updatecount == 0)
            {
                oldPoseTuple = this.currentRobotPose;
                return;
            }
            if (oldPoseTuple!=currentRobotPose)
            {
                RealiseRobotPose(currentRobotPose);
                oldPoseTuple = currentRobotPose;
            }
        }

        RobotPose oldPoseTuple;
        int updatecount;
        void Update()
        {
            DoRobotPose();
            updatecount++;
        }
    }
}