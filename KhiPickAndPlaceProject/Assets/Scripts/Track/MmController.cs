using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KhiDemo
{
    public enum RobStatus { busy,idle }
    public class MmController : MonoBehaviour
    {


        public MmMode mmMode = MmMode.None;
        public MmSubMode mmSubMode = MmSubMode.None;
        public MmBoxMode mmBoxMode = MmBoxMode.Fake;

        public MmRobot mmRobot = null;
        public MmTray mmtray = null;


        MagneMotion magmo;
        MmTable mmt;

        [Header("Simulation")]
        public bool processStep;
        [Space(10)]
        public bool reverseTrayRail;
        public bool doubleSpeed;
        public bool halfSpeed;

        float srobsec = 0.4f;
        float lrobsec = 1.0f;

        public RobStatus robstatus;

        public void Init(MagneMotion magmo)
        {
            this.magmo = magmo;
            mmt = magmo.mmt;
            mmtray = magmo.mmtray;
            mmRobot = magmo.mmRobot;
            robstatus = RobStatus.idle;
        }
        // Start is called before the first frame update
        void Start()
        {
        }

        public void SetMode(MmMode newMode)
        {
            mmMode = newMode;            switch (newMode)
            {
                default:
                case MmMode.Echo:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Fake;
                    magmo.boxForm = MmBox.BoxForm.Prefab;
                    magmo.echoMovements = true;
                    magmo.publishMovements = false;
                    mmRobot.InitRobotBoxState(true);
                    mmtray.InitAllLoadstate(true,nbox:12); // doesn't really matter - all gets overwritten
                    mmt.SetupSleds(SledLoadDistrib.alternateLoadedUnloaded, SledSpeedDistrib.fixedValue, 0);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
                case MmMode.SimuRailToRail:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithSphere;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(false, nbox: 10);
                    mmt.SetupSleds(SledLoadDistrib.alternateLoadedUnloaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
                case MmMode.StartRailToTray:
                    mmSubMode = MmSubMode.RailToTray;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithSphere;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(false, nbox: 10);
                    mmt.SetupSleds(SledLoadDistrib.allLoaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
                case MmMode.StartTrayToRail:
                    mmSubMode = MmSubMode.TrayToRail;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithSphere;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    mmRobot.InitRobotBoxState(false);
                    mmtray.InitAllLoadstate(true, nbox: 10);
                    mmt.SetupSleds(SledLoadDistrib.allUnloaded, SledSpeedDistrib.alternateHiLo, 0.5f);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
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

        IEnumerator TransferBoxFromSledToRobot(MmSled sled,MmRobot rob)
        {
            yield return new WaitUntil(() => robstatus == RobStatus.idle);
            robstatus = RobStatus.busy;
            mmRobot.RealiseRobotPose(RobotPose.fcartup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.fcartdn);
            yield return new WaitForSeconds(lrobsec);
            var box = sled.DetachhBoxFromSled();
            if (box != null)
            {
                rob.AttachBoxToRobot(box);
            }
            mmRobot.RealiseRobotPose(RobotPose.fcartup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.rest);
            yield return new WaitForSeconds(lrobsec);
            robstatus = RobStatus.idle;
        }

        IEnumerator TransferBoxFromRobotToSled(MmRobot rob, MmSled sled)
        {
            yield return new WaitUntil(() => robstatus == RobStatus.idle);
            robstatus = RobStatus.busy;
            mmRobot.RealiseRobotPose(RobotPose.ecartup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.ecartdn);
            yield return new WaitForSeconds(lrobsec);
            var box = rob.DetachhBoxFromRobot();
            if (box != null)
            {
                sled.AttachBoxToSled(box);
            }
            mmRobot.RealiseRobotPose(RobotPose.ecartup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.rest);
            yield return new WaitForSeconds(lrobsec);
            robstatus = RobStatus.idle;
        }

        IEnumerator TransferBoxFromTrayToRobot((int,int) key, MmRobot rob)
        {
            yield return new WaitUntil(() => robstatus == RobStatus.idle);
            robstatus = RobStatus.busy;
            var (poseup, posedn) = mmRobot.GetPoses(key);
            mmRobot.RealiseRobotPose(poseup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(posedn);
            yield return new WaitForSeconds(lrobsec);
            var box = mmtray.DetachhBoxFromTraySlot(key);
            if (box != null)
            {
                rob.AttachBoxToRobot(box);
            }
            mmRobot.RealiseRobotPose(poseup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.rest);
            yield return new WaitForSeconds(lrobsec);
            robstatus = RobStatus.idle;
        }

        IEnumerator TransferBoxFromRobotToTray(MmRobot rob, (int, int) key )
        {
            yield return new WaitUntil(() => robstatus == RobStatus.idle);
            robstatus = RobStatus.busy;
            var (poseup, posedn) = mmRobot.GetPoses(key);
            mmRobot.RealiseRobotPose(poseup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(posedn);
            yield return new WaitForSeconds(lrobsec);
            var box = rob.DetachhBoxFromRobot();
            if (box != null)
            {
                mmtray.AttachBoxToTraySlot(key, box);
            }
            mmRobot.RealiseRobotPose(poseup);
            yield return new WaitForSeconds(srobsec);
            mmRobot.RealiseRobotPose(RobotPose.rest);
            yield return new WaitForSeconds(lrobsec);
            robstatus = RobStatus.idle;
        }

        public enum TranferType { SledToRob, RobToSled, TrayToRob, RobToTray }


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
                            StartCoroutine(TransferBoxFromSledToRobot(sled,rob));
                            //var box = sled.DetachhBoxFromSled();
                            //if (box != null)
                            //{
                            //    rob.AttachBoxToRobot(box);
                            //}
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
                            StartCoroutine(TransferBoxFromRobotToSled(rob,sled));
                            //var box = rob.DetachhBoxFromRobot();
                            //if (box != null)
                            //{
                            //    sled.AttachBoxToSled(box);
                            //}
                            break;
                    }
                    break;
                default:
                    Debug.LogError("SledTransferBox - Wrong Function");
                    break;
            }
        }

        public void TrayTransferBox(TranferType tt, (int, int) key)
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
                            StartCoroutine(TransferBoxFromTrayToRobot(key, rob));
                            //var box = mmtray.DetachhBoxFromTraySlot(key);
                            //if (box != null)
                            //{
                            //    rob.AttachBoxToRobot(box);
                            //}
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
                            StartCoroutine(TransferBoxFromRobotToTray(rob,key));
                            //var box = rob.DetachhBoxFromRobot();
                            //if (box != null)
                            //{
                            //    mmtray.AttachBoxToTraySlot(key, box);
                            //}
                            break;
                    }
                    break;
                default:
                    Debug.LogError("TrayTransferBox - Wrong Function");
                    break;
            }
        }

        public bool CheckIfOkayForNextProcessStep()
        {
            if (robstatus!= RobStatus.idle)
            {
                return false;
            }
            var (nloadestopped, nunloadedstopped) = mmt.CountStoppedSleds();
            var (_, nTray, nRob, nSled) = MmBox.CountBoxStatus();
            //Debug.Log($"CountBoxStatus nTray:{nTray} nRob:{nRob} nSled:{nSled}");
            if (mmSubMode == MmSubMode.RailToTray && nSled == 0 && nRob == 0)
            {
                mmSubMode = MmSubMode.TrayToRail;
                //Debug.Log($"   Switched to {mmSubMode}");
            }
            else if (mmSubMode == MmSubMode.TrayToRail && nTray == 0 && nRob == 0)
            {
                mmSubMode = MmSubMode.RailToTray;
                //Debug.Log($"   Switched to {mmSubMode}");
            }
            switch (mmMode)
            {
                case MmMode.SimuRailToRail:
                    {
                        var rv = (nloadestopped > 0 && nunloadedstopped > 0);
                        return rv;
                    }
                case MmMode.StartRailToTray:
                case MmMode.StartTrayToRail:
                    {

                        switch (mmSubMode)
                        {
                            case MmSubMode.RailToTray:
                                var rv1 = nloadestopped > 0 || nRob > 0;
                                return rv1;
                            case MmSubMode.TrayToRail:
                                var rv2 = nunloadedstopped > 0 && (nTray+nRob)>0;
                                return rv2;
                        }
                    }
                    break;
            }
            return false;
        }

        public void ProcessStep()
        {
            //if (!processStep) return;
            //processStep = false;
            if (!CheckIfOkayForNextProcessStep()) return;
            //Debug.Log($"ProcessStep mode:{mmMode}");

            var errhead = $"MagneMotion.demoStep - {mmMode}";
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
                            SledTransferBox(TranferType.SledToRob, s);
                        }
                        else
                        {
                            var s = mmt.FindStoppedSled(neededLoadState: false);
                            if (s == null)
                            {
                                Debug.LogWarning($"{errhead} - cound not find stoppedsled with loadState {false} to unload");
                                return;
                            }
                            SledTransferBox(TranferType.RobToSled, s);
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
                        break;
                    }
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


        public void PhysicsStep()
        {
            AdjustSpeed();
            ReverseTrayRail();

        }

        // Update is called once per frame
        void Update()
        {
            ProcessStep();
        }
    }
}