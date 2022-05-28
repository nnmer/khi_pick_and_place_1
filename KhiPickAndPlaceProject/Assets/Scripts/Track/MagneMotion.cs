using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public enum MmBoxMode { Fake, Real }

    public enum MmSegForm { None, Straight, Curved }

    public class MagneMotion : MonoBehaviour
    {
        public MnTable mmt;
        public GameObject robmodel;
        public GameObject vgrip;
        public MmBoxMode mmBoxMode = MmBoxMode.Fake;
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
                var urdfRobot = FindObjectOfType<Unity.Robotics.UrdfImporter.UrdfRobot>();
                if (urdfRobot==null)
                {
                    Debug.LogError("Robmodel not set in Magnemotion table");
                }
                else
                {
                    robmodel = urdfRobot.gameObject;
                }
            }

            if (robmodel!=null && planner!=null )
            { 
                mmt.AddBoxToRobot(planner.vgriptrans);
            }

            var mmgo = mmt.SetupGeometry(addPathMarkers: addPathMarkers, addPathSleds: addPathSledsOnStartup, positionOnFloor: positionOnFloor);
            mmgo.transform.SetParent(gameObject.transform, false);

            mmtray = FindObjectOfType<MmTray>();
            if (mmtray != null)
            {
                mmtray.Init(this);
            }
        }



        MmSled.SledForm oldsledForm;
        MmBox.BoxForm oldboxForm;
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


        int updatecount = 0;
        // Update is called once per frame
        void Update()
        {
            ChangeSledFormIfRequested();
            ChangeBoxFormIfRequested();
            updatecount++;
        }
    }
}