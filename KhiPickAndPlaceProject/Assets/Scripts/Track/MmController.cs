using System;
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

        public void Clear()
        {
            mmtray.Clear();
            mmt.Clear();// doesn't do anything
            mmRobot.Clear();
            MmBox.Clear();
            GC.Collect();
        }
        public void SetMode(MmMode newMode,bool clear=false)
        {
            if (clear)
            {
                Clear();
            }
            mmMode = newMode;
            switch (newMode)
            {
                default:
                case MmMode.EchoNew:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Fake;
                    magmo.boxForm = MmBox.BoxForm.Prefab;
                    magmo.echoMovements = true;
                    magmo.publishMovements = false;
                    mmt.SetupSledSpeeds(SledSpeedDistrib.fixedValue, 0);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);

                    mmRobot.InitRobotBoxState(startLoadState:true, usePools: true);
                    mmt.SetupSledLoads(SledLoadDistrib.alternateLoadedUnloaded, usePools: true);
                    mmtray.InitAllLoadstate(nbox: 12, usePools: true); // doesn't really matter - all gets overwritten
                    break;
                case MmMode.Echo:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Fake;
                    magmo.boxForm = MmBox.BoxForm.Prefab;
                    magmo.echoMovements = true;
                    magmo.publishMovements = false;
                    mmRobot.InitRobotBoxState(startLoadState: true);
                    mmt.SetupSledSpeeds( SledSpeedDistrib.fixedValue, 0);
                    mmt.SetupSledLoads(SledLoadDistrib.alternateLoadedUnloaded);
                    mmtray.InitAllLoadstate(nbox: 12); // doesn't really matter - all gets overwritten
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
                case MmMode.SimNew:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithMarkerCube;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    mmt.SetupSledSpeeds(SledSpeedDistrib.alternateHiLo, 0.5f);

                    mmRobot.InitRobotBoxState(startLoadState: false, usePools: true);
                    var nloadedSleds = mmt.CountLoadedSleds();
                    mmtray.InitAllLoadstate(nbox: 10 - nloadedSleds, usePools: true);
                    mmt.SetupSledLoads(SledLoadDistrib.alternateLoadedUnloaded, usePools: true);
                    break;
                case MmMode.SimuRailToRail:
                    mmSubMode = MmSubMode.None;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithMarkerCube;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    mmt.SetupSledSpeeds(SledSpeedDistrib.alternateHiLo, 0.5f);

                    mmRobot.InitRobotBoxState(startLoadState: false);
                    var nloadedSleds1 = mmt.CountLoadedSleds();
                    mmtray.InitAllLoadstate(nbox: 10-nloadedSleds1);
                    mmt.SetupSledLoads(SledLoadDistrib.alternateLoadedUnloaded);
                    break;
                case MmMode.StartRailToTray:
                    mmSubMode = MmSubMode.RailToTray;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithMarkerCube;
                    mmRobot.InitRobotBoxState(startLoadState: false);
                    mmt.SetupSledSpeeds( SledSpeedDistrib.alternateHiLo, 0.5f);
                    mmt.SetupSledLoads(SledLoadDistrib.allLoaded);
                    mmtray.InitAllLoadstate(nbox: 10);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
                case MmMode.StartTrayToRail:
                    mmSubMode = MmSubMode.TrayToRail;
                    mmBoxMode = MmBoxMode.Real;
                    magmo.boxForm = MmBox.BoxForm.PrefabWithMarkerCube;
                    magmo.echoMovements = false;
                    magmo.publishMovements = true;
                    mmRobot.InitRobotBoxState(startLoadState: false);
                    mmt.SetupSledSpeeds( SledSpeedDistrib.alternateHiLo, 0.5f);
                    mmt.SetupSledLoads(SledLoadDistrib.allUnloaded);
                    mmtray.InitAllLoadstate(nbox: 10);
                    magmo.mmRobot.RealiseRobotPose(RobotPose.rest);
                    break;
            }
            CheckConsistency();
        }

        public void AdjustRobotSpeedFactor(float fak)
        {
            srobsec /= fak;
            lrobsec /= fak;
        }


        public void DoReverseTrayRail()
        {
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

        public void ReverseTrayRail()
        {
            if (!reverseTrayRail) return;
            reverseTrayRail = false;
            DoReverseTrayRail();
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
            if (mmMode == MmMode.SimuRailToRail)
            {
                mmRobot.RealiseRobotPose(RobotPose.restr2r);
            }
            else
            {
                mmRobot.RealiseRobotPose(RobotPose.rest);
            }
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
            if (mmMode == MmMode.SimuRailToRail)
            {
                mmRobot.RealiseRobotPose(RobotPose.restr2r);
            }
            else
            {
                mmRobot.RealiseRobotPose(RobotPose.rest);
            }
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
                            break;
                    }
                    break;
                default:
                    Debug.LogError("TrayTransferBox - Wrong Function");
                    break;
            }
        }

        public bool CheckConsistency()
        {
            if (mmMode== MmMode.Echo || mmMode==MmMode.None)
            {
                if (mmBoxMode == MmBoxMode.Real)
                {
                    var emsg2 = $"ChkCon error: unsupported mode mode:{mmMode} boxMode:{mmBoxMode}";
                    Debug.LogError(emsg2);
                    return false;
                }
                return true;
            }
            if (mmBoxMode== MmBoxMode.Fake)
            {
                var emsg1 = $"ChkCon error: unsupported mode mode:{mmMode} boxMode:{mmBoxMode}";
                Debug.LogError(emsg1);
                return false;
            }
            bool rv = false;
            var nTray = mmtray.CountLoaded();
            var nSled = mmt.CountLoadedSleds();
            var nRob = mmRobot.IsLoaded() ? 1 : 0;
            var emsg = $"ChkCon error: boxMode:{mmBoxMode} ntray:{nTray} nsled:{nSled} nrob:{nRob}";
            rv = (nTray + nSled + nRob)==10;
            if (!rv)
            {
                Debug.LogError($"{emsg} does not sum to 10");
                //magmo.stopSimulation = true;
                
            }
            var (nFree, nTrayCbs, nRobCbs, nSledCbs) = MmBox.CountBoxStatus();
            if (nFree>0)
            {
                Debug.LogError($"Free box got away:{nFree}");
                (nFree, nTrayCbs, nRobCbs, nSledCbs) = MmBox.CountBoxStatus();
               // magmo.stopSimulation = true;
            }
            if (nTrayCbs!=nTray)
            {
                Debug.LogError($"{emsg}  nTrayCbs:{nTrayCbs} not equal to nTray");
                (nFree, nTrayCbs, nRobCbs, nSledCbs) = MmBox.CountBoxStatus();
                //magmo.stopSimulation = true;
            }
            if (nRobCbs != nRob)
            {
                Debug.LogError($"{emsg}  nRobCbs:{nRobCbs} not equal to nRob");
                (nFree, nTrayCbs, nRobCbs, nSledCbs) = MmBox.CountBoxStatus();
                //magmo.stopSimulation = true;
            }
            if (nSledCbs != nSled)
            {
                Debug.LogError($"{emsg}  nSledCbs:{nSledCbs} not equal to nSled");
                (nFree, nTrayCbs, nRobCbs, nSledCbs) = MmBox.CountBoxStatus();
                //magmo.stopSimulation = true;
            }
            return rv;
        }

        public bool CheckIfOkayForNextProcessStep()
        {
            var rv = false;
            var (nloadedstopped, nunloadedstopped) = mmt.CountStoppedSleds();
            var (_, nTray, nRob, nSled) = MmBox.CountBoxStatus();
            try
            {
                if (robstatus != RobStatus.idle)
                {
                    return rv;
                }
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
                            rv = (nloadedstopped > 0 && nunloadedstopped > 0);
                            return rv;
                        }
                    case MmMode.StartRailToTray:
                    case MmMode.StartTrayToRail:
                        {

                            switch (mmSubMode)
                            {
                                case MmSubMode.RailToTray:
                                    rv = nloadedstopped > 0 || nRob > 0;
                                    return rv;
                                case MmSubMode.TrayToRail:
                                    rv = nunloadedstopped > 0 && (nTray + nRob) > 0;
                                    return rv;
                            }
                        }
                        break;
                }
                return rv;
            }
            finally
            {
                var msg = $"CIOFNP {mmMode} {mmSubMode} - nls:{nloadedstopped} nus:{nunloadedstopped} nTray:{nTray} nRob:{nRob} nSled:{nSled} rv:{rv}";
                Debug.Log(msg);
            }
        }

        public void ProcessStep()
        {
            CheckConsistency();
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
            if (!magmo.stopSimulation)
            {
                ProcessStep();
            }
        }
    }
}