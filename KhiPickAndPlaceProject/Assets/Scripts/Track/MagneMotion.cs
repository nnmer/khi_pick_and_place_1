using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public enum MmBoxMode { Fake, Real }

    public enum MmSegForm { None, Straight, Curved }

    public enum MmMode {  None, Echo, Simulate }

    public class MagneMotion : MonoBehaviour
    {
        public MnTable mmt;
        public GameObject robmodel;
        public MmBoxMode mmBoxMode = MmBoxMode.Fake;
        public MmMode mmMode = MmMode.None;
        public GameObject mmtgo;


        public bool addPathMarkers = false;
        public bool addPathSledsOnStartup = true;
        public bool positionOnFloor = false;

        public MmTray mmtray = null;
        public Rs007TrajectoryPlanner planner = null;
        public MmSled.SledForm sledForm = MmSled.SledForm.Prefab;
        public MmBox.BoxForm boxForm = MmBox.BoxForm.Prefab;


        public Dictionary<string, MmSled> SledDict = new Dictionary<string, MmSled>();

        // Start is called before the first frame update
        void Start()
        {
            planner = GameObject.FindObjectOfType<Rs007TrajectoryPlanner>();

            mmtgo = new GameObject("MmTable");
            mmt = mmtgo.AddComponent<MnTable>();
            mmt.Init(this);

            MmPathSeg.InitDicts();
            mmt.MakeMsftDemoMagmo();
            //mmt.MakeSimplePath();


            // Initialize Robot
            if (robmodel == null)
            {
                var mmRobot = FindObjectOfType<MmRobot>();
                if (mmRobot==null)
                {
                    Debug.LogError("Robmodel not set in Magnemotion table");
                }
                else
                {
                    robmodel = mmRobot.gameObject;
                }
            }

            if (robmodel!=null && planner!=null )
            { 
                mmt.AddBoxToRobot(planner.vgriptrans);
            }

            var mmgo = mmt.SetupGeometry(addPathMarkers: addPathMarkers, positionOnFloor: positionOnFloor);
            mmgo.transform.SetParent(gameObject.transform, false);
            if (addPathSledsOnStartup)
            {
                mmt.AddSleds();
            }
            SetMode(mmMode);

            mmtray = FindObjectOfType<MmTray>();
            if (mmtray != null)
            {
                mmtray.Init(this);
            }
        }



        public void SetMode(MmMode mmMode)
        {
            switch(mmMode)
            {
                default:
                case MmMode.Echo:
                    mmt.SetSledSpeeds( SledSpeedDistribution.fixedValue, 0);
                    break;
                case MmMode.Simulate:
                    mmt.SetSledSpeeds(SledSpeedDistribution.alternateHiLo, 2);
                    break;
            }
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
                SetMode(mmMode);
                oldmmMode = mmMode;
            }
        }

        int updatecount = 0;
        // Update is called once per frame
        void Update()
        {
            ChangeSledFormIfRequested();
            ChangeBoxFormIfRequested();
            ChangeModeIfRequested();
            updatecount++;
        }
    }
}