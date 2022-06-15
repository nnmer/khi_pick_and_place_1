using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public enum MmBoxMode { Fake, Real }

    public enum MmSegForm { None, Straight, Curved }


    public enum MmTableStyle {  MftDemo, Simple }

    public enum MmMode { None, EchoNew, SimNew, Echo, SimuRailToRail, StartRailToTray, StartTrayToRail }
    public enum MmSubMode { None, RailToTray, TrayToRail }

    public class MagneMotion : MonoBehaviour
    {
        public MmController mmctrl;
        public MmTable mmt;
        public MmTableStyle mmTableStyle = MmTableStyle.MftDemo;
        public GameObject mmtgo;
        public GameObject mmtctrlgo;


        public bool addPathMarkers = false;
        public bool addPathSledsOnStartup = true;
        public bool positionOnFloor = false;

        public MmRobot mmRobot = null;
        public MmTray mmtray = null;
        public ROSConnection ros = null;
        public Rs007TrajectoryPlanner planner = null;
        public MmSled.SledForm sledForm = MmSled.SledForm.Prefab;
        public MmBox.BoxForm boxForm = MmBox.BoxForm.Prefab;

        public MmMode mmMode = MmMode.None;

        public bool stopSimulation = false;

        public bool echoMovements = true;
        public bool publishMovements = false;
        public float publishInterval = 0.1f;



        private void Awake()
        {
            // we need this before any ros-dependent component starts
            ros = ROSConnection.GetOrCreateInstance();

        }
        // Start is called before the first frame update
        void Start()
        {
            planner = GameObject.FindObjectOfType<Rs007TrajectoryPlanner>();

            mmtgo = new GameObject("MmTable");
            mmt = mmtgo.AddComponent<MmTable>();
            mmt.Init(this);

            MmBox.AllocatePools(this);

            MmPathSeg.InitDicts();


            switch (mmTableStyle)
            {
                default:
                case MmTableStyle.MftDemo:
                    mmt.MakeMsftDemoMagmo();
                    break;
                case MmTableStyle.Simple:
                    mmt.MakeSimplePath();
                    break;
            }

            // Initialize Robot
            mmRobot = FindObjectOfType<MmRobot>();
            if (mmRobot==null)
            {
                Debug.LogError("Robmodel not set in Magnemotion table");
            }


            var mmgo = mmt.SetupGeometry(addPathMarkers: addPathMarkers, positionOnFloor: positionOnFloor);
            mmgo.transform.SetParent(transform, false);
            if (addPathSledsOnStartup)
            {
                mmt.AddSleds();
            }

            mmtray = FindObjectOfType<MmTray>();
            if (mmtray != null)
            {
                mmtray.Init(this);
            }

            // needs ot go last
            mmtctrlgo = new GameObject("MmCtrl");
            mmctrl = mmtctrlgo.AddComponent<MmController>();
            mmctrl.Init(this);
            mmctrl.SetMode(mmMode);
        }



        MmSled.SledForm oldsledForm;
        void ChangeSledFormIfRequested()
        {
            if (updatecount == 0)
            {
                oldsledForm = sledForm;
            }
            if (sledForm != oldsledForm)
            {
                var sleds = FindObjectsOfType<MmSled>();
                foreach (var sled in sleds)
                {
                    sled.ConstructForm(sledForm);
                }
                oldsledForm = sledForm;
            }
        }

        MmBox.BoxForm oldboxForm;
        void ChangeBoxFormIfRequested()
        {
            if (updatecount == 0)
            {
                oldboxForm = boxForm;
            }
            if (boxForm != oldboxForm)
            {
                var boxes = FindObjectsOfType<MmBox>();
                foreach (var box in boxes)
                {
                    box.ConstructForm(boxForm);
                }
                oldboxForm = boxForm;
            }
        }

        MmMode oldmmMode;
        void ChangeModeIfRequested()
        {
            if (updatecount == 0)
            {
                oldmmMode = mmMode;
            }
            if (oldmmMode != mmMode)
            {
                mmctrl.SetMode(mmMode);
                oldmmMode = mmMode;
            }
        }




        float lastPub = 0;

        public void PhysicsStep()
        {
            //var fps = 1 / Time.deltaTime;
            //Debug.Log($"MagneMotion Simstep time:{Time.time:f3} deltatime:{Time.deltaTime:f3} fps:{fps:f2}");

            if (!stopSimulation)
            {
                mmctrl.PhysicsStep();
                mmt.PhysicsStep();
            }
            if (publishMovements && Time.time-lastPub>this.publishInterval)
            {
                lastPub = Time.time;
                mmRobot.PublishJoints();
                mmtray.PublishTray();
                mmt.PublishSleds();
            }
        }

        public void Quit()
        {
            Application.Quit();
#if UNITY_EDITOR
            EditorApplication.ExecuteMenuItem("Edit/Play");// this makes the editor quit playing
#endif
        }


        float ctrlChitTime = 0;
        float ctrlMhitTime = 0;
        float ctrlQhitTime = 0;
        float ctrlShitTime = 0;
        float ctrlDhitTime = 0;
        float ctrlEhitTime = 0;
        float ctrlThitTime = 0;
        float ctrlLhitTime = 0;
        float ctrlFhitTime = 0;
        float ctrlRhitTime = 0;
        float F5hitTime = 0;
        float F6hitTime = 0;
        float F10hitTime = 0;
        public void KeyProcessing()
        {
            var ctrlhit = Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl);
            if (ctrlhit && Input.GetKeyDown(KeyCode.Q))
            {
                Debug.Log("Hit Ctrl-Q");
                if ((Time.time - ctrlQhitTime) < 1)
                {
                    Debug.Log("Hit it twice so quitting: Application.Quit()");
                    Quit();
                }
                // CTRL + Q - 
                ctrlQhitTime = Time.time;
            }
            if (((Time.time - F5hitTime) > 0.5) && Input.GetKeyDown(KeyCode.F5))
            {
                Debug.Log("F5 - Request Total Refresh");
            }
            if (((Time.time - F6hitTime) > 0.5) && Input.GetKeyDown(KeyCode.F6))
            {
                Debug.Log("F6 - Request Go Refresh");
            }
            if (((Time.time - F10hitTime) > 1) && Input.GetKeyDown(KeyCode.F10))
            {
                Debug.Log("F10 - Options");
                // uiman.optpan.TogglePanelState();
                //this.RequestRefresh("F5 hit", totalrefresh: true);
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.C))
            {
                Debug.Log("Hit Ctrl-C - interrupting");
                // CTRL + C
                ctrlChitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.D))
            {
                Debug.Log("Hit LCtrl-D");
                ctrlDhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.E))
            {
                Debug.Log("Hit LCtrl-E");
                mmctrl.SetMode(MmMode.Echo,clear:true);

                ctrlEhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.T))
            {
                Debug.Log("Hit LCtrl-T");
                mmctrl.SetMode(MmMode.StartTrayToRail, clear: true);
                ctrlEhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.L))
            {
                Debug.Log("Hit LCtrl-:");
                mmctrl.SetMode(MmMode.SimuRailToRail, clear: true);
                ctrlLhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.R))
            {
                Debug.Log("Hit LCtrl-R");
                mmctrl.DoReverseTrayRail();
                ctrlEhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.F))
            {
                Debug.Log("Hit LCtrl-F");
                mmt.AdjustSledSpeedFactor(2);
                mmctrl.AdjustRobotSpeedFactor(2);
                ctrlFhitTime = Time.time;
            }
            if (ctrlhit && Input.GetKeyDown(KeyCode.S))
            {
                Debug.Log("Hit LCtrl-S");
                mmt.AdjustSledSpeedFactor(0.5f);
                mmctrl.AdjustRobotSpeedFactor(0.5f);
                ctrlShitTime = Time.time;
            }
        }


        int fupdatecount = 0;
        // Update is called once per frame
        void FixedUpdate()
        {
            PhysicsStep();
            fupdatecount++;
        }

        int updatecount = 0;
        // Update is called once per frame
        void Update()
        {
            KeyProcessing();
            ChangeSledFormIfRequested();
            ChangeBoxFormIfRequested();
            ChangeModeIfRequested();
            updatecount++;
        }
    }
}