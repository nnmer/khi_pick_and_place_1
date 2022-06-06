using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public enum MmBoxMode { Fake, Real }

    public enum MmSegForm { None, Straight, Curved }

    public enum MmMode {  None, Echo, SimuRailToRail, StartRailToTray, StartTrayToRail}
    public enum MmSubMode { None, RailToTray, TrayToRail }

    public enum MmTableStyle {  MftDemo, Simple }

    public class MagneMotion : MonoBehaviour
    {
        public MnTable mmt;
        public MmRobot mmRobot;
        public MmBoxMode mmBoxMode = MmBoxMode.Fake;
        public MmMode mmMode = MmMode.None;
        public MmSubMode mmSubMode = MmSubMode.None;
        public MmTableStyle mmTableStyle = MmTableStyle.MftDemo;
        public GameObject mmtgo;


        public bool addPathMarkers = false;
        public bool addPathSledsOnStartup = true;
        public bool positionOnFloor = false;

        public MmTray mmtray = null;
        public Rs007TrajectoryPlanner planner = null;
        public MmSled.SledForm sledForm = MmSled.SledForm.Prefab;
        public MmBox.BoxForm boxForm = MmBox.BoxForm.Prefab;

        [Header("Simulation")]
        public bool simStep;
        [Space(10)]
        public bool reverseTrayRail;
        public bool doubleSpeed;
        public bool halfSpeed;


        public Dictionary<string, MmSled> SledDict = new Dictionary<string, MmSled>();

        // Start is called before the first frame update
        void Start()
        {
            planner = GameObject.FindObjectOfType<Rs007TrajectoryPlanner>();

            mmtgo = new GameObject("MmTable");
            mmt = mmtgo.AddComponent<MnTable>();
            mmt.Init(this);

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
            SetMode(mmMode);
        }
        public void SetMode(MmMode newMode)
        {
            switch(newMode)
            {
                default:
                case MmMode.Echo:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Fake;
                    mmRobot.InitRobotBoxState(true);
                    mmtray.InitAllLoadstate(true); // doesn't really matter - all gets overwritten
                    mmt.SetupSleds(SledLoadDistrib.alternateLoadedUnloaded, SledSpeedDistrib.fixedValue, 0);
                    break;
                case MmMode.SimuRailToRail:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Real;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(false);
                    mmt.SetupSleds(SledLoadDistrib.alternateLoadedUnloaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    break;
                case MmMode.StartRailToTray:
                    mmSubMode = MmSubMode.RailToTray;
                    mmBoxMode = MmBoxMode.Real;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(false);
                    mmt.SetupSleds(SledLoadDistrib.allLoaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    break;
                case MmMode.StartTrayToRail:
                    mmSubMode = MmSubMode.TrayToRail;
                    mmBoxMode = MmBoxMode.Fake;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(true);
                    mmt.SetupSleds(SledLoadDistrib.allUnloaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    break;
            }
        }

        public void ReverseTrayRail()
        {
            if (!reverseTrayRail) return;
            reverseTrayRail = false;
            if (mmSubMode == MmSubMode.RailToTray)
            {
                Debug.Log($"Reversing submode to {MmSubMode.TrayToRail}");
                mmSubMode = MmSubMode.TrayToRail;
            }
            else if (mmSubMode == MmSubMode.TrayToRail)
            {
                Debug.Log($"Reversing submode to {MmSubMode.RailToTray}");
                mmSubMode = MmSubMode.RailToTray;
            }
        }

        public enum TranferType {  SledToRob, RobToSled, TrayToRob, RobToTray }

        public void SledTransferBox(TranferType tt, MmSled sled)
        {
            var rob = mmRobot;
            switch (tt)
            {
                case TranferType.SledToRob:
                    switch (mmBoxMode)
                    {
                        case MmBoxMode.Fake:
                            sled.SetLoadState(false);
                            rob.ActivateRobBox(true);
                            break;
                        case MmBoxMode.Real:
                            var box = sled.DetachhBoxFromSled();
                            if (box != null)
                            {
                                rob.AttachBoxToRobot(box);
                            }
                            break;
                    }
                    break;
                case TranferType.RobToSled:
                    switch (mmBoxMode)
                    {
                        case MmBoxMode.Fake:
                            sled.SetLoadState(true);
                            rob.ActivateRobBox(false);
                            break;
                        case MmBoxMode.Real:
                            var box = rob.DetachhBoxFromRobot();
                            if (box != null)
                            {
                                sled.AttachBoxToSled(box);
                            }
                            break;
                    }
                    break;
                default:
                    Debug.LogError("SledTransferBox - Wrong Function");
                    break;
            }
        }

        public void TrayTransferBox(TranferType tt, (int,int) key)
        {
            var rob = mmRobot;
            switch (tt)
            {
                case TranferType.TrayToRob:

                    switch (mmBoxMode)
                    {
                        case MmBoxMode.Fake:
                            mmtray.SetVal(key, false);
                            rob.ActivateRobBox(true);
                            break;
                        case MmBoxMode.Real:
                            var box = mmtray.DetachhBoxFromTraySlot(key);
                            if (box != null)
                            {
                                rob.AttachBoxToRobot(box);
                            }
                            break;
                    }
                    break;
                case TranferType.RobToTray:
                    switch (mmBoxMode)
                    {
                        case MmBoxMode.Fake:
                            mmtray.SetVal(key, true);
                            rob.ActivateRobBox(false);
                            break;
                        case MmBoxMode.Real:
                            var box = rob.DetachhBoxFromRobot();
                            if (box != null)
                            {
                                mmtray.AttachBoxToTraySlot(key, box);
                            }
                            break;
                    }
                    break;
                default:
                    Debug.LogError("TrayTransferBox - Wrong Function");
                    break;
            }
        }


        public void SimStep()
        {
            if (!simStep) return;
            simStep = false;
            var errhead = $"MagneMotion.SimStep - {mmMode}";
            var rob = mmRobot;
            switch (mmMode)
            {
                case MmMode.SimuRailToRail:
                    {
                        if (!rob.loadState)
                        {
                            var s = mmt.FindStoppedSled(neededLoadState: true);
                            if (s == null)
                            {
                                Debug.LogWarning($"{errhead} - cound not find stoppedsled with loadState {true} to load");
                                return;
                            }
                            SledTransferBox(TranferType.SledToRob,s);
                        }
                        else
                        {
                            var s = mmt.FindStoppedSled(neededLoadState: false);
                            if (s == null)
                            {
                                Debug.LogWarning($"{errhead} - cound not find stoppedsled with loadState {false} to unload");
                                return;
                            }
                            SledTransferBox(TranferType.RobToSled,s);
                        }
                        break;
                    }
                case MmMode.StartRailToTray:
                case MmMode.StartTrayToRail:
                    {
                        switch (mmSubMode)
                        {
                            case MmSubMode.RailToTray:
                                {
                                    if (!rob.loadState)
                                    {
                                        var s = mmt.FindStoppedSled(neededLoadState: true);
                                        if (s == null)
                                        {
                                            Debug.LogWarning($"{errhead}  - cound not find stoppedsled with loadState {true} to unload");
                                            return;
                                        }
                                        SledTransferBox(TranferType.SledToRob, s);
                                    }
                                    else
                                    {
                                        var (found, key) = mmtray.FindFirst(seekLoadState: false);
                                        if (!found)
                                        {
                                            Debug.LogWarning($"{errhead}  - cound not find empty tray slot");
                                            return;
                                        }
                                        if (found)
                                        {
                                            TrayTransferBox(TranferType.RobToTray, key);
                                        }
                                    }
                                    break;
                                }
                            case MmSubMode.TrayToRail:
                                {
                                    if (!rob.loadState)
                                    {
                                        var (found, key) = mmtray.FindFirst(seekLoadState: true);
                                        if (!found)
                                        {
                                            Debug.LogWarning($"{errhead}  - cound not find loaded tray slot to unload");
                                            return;
                                        }
                                        TrayTransferBox(TranferType.TrayToRob, key);
                                    }
                                    else
                                    {
                                        var s = mmt.FindStoppedSled(neededLoadState: false);
                                        if (s == null)
                                        {
                                            Debug.LogWarning($"{errhead}  - cound not find stoppedsled with loadState {false} to load");
                                            return;
                                        }
                                        SledTransferBox(TranferType.RobToSled, s);
                                    }
                                    break;
                                }
                        }
                        var (_, nTray, nRob, nSled) = MmBox.CountBoxStatus();
                        Debug.Log($"CountBoxStatus nTray:{nTray} nRob:{nRob} nSled:{nSled}");
                        if (mmSubMode== MmSubMode.RailToTray && nSled==0 && nRob==0)
                        {
                            mmSubMode = MmSubMode.TrayToRail;
                            Debug.Log($"   Switched to {mmSubMode}");
                        }
                        else if (mmSubMode == MmSubMode.TrayToRail && nTray==0 && nRob==0)
                        {
                            mmSubMode = MmSubMode.RailToTray;
                            Debug.Log($"   Switched to {mmSubMode}");
                        }
                        break;
                    }
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

        void AdjustSpeed()
        {
            if (doubleSpeed)
            {
                doubleSpeed = false;
                mmt.AdjustSledSpeedFactor(2f);
            }
            if (halfSpeed)
            {
                halfSpeed = false;
                mmt.AdjustSledSpeedFactor(0.5f);
            }
        }

        int updatecount = 0;
        // Update is called once per frame
        void Update()
        {
            ChangeSledFormIfRequested();
            ChangeBoxFormIfRequested();
            ChangeModeIfRequested();
            ReverseTrayRail();
            AdjustSpeed();
            SimStep();
            updatecount++;
        }
    }
}