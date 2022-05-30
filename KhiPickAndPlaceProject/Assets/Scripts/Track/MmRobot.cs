using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KhiDemo
{
    public class MmRobot : MonoBehaviour
    {
        public bool loadState;
        public MagneMotion magmo;
        public Transform vgriptrans;
        public bool transferBox;
        public GameObject boxgo;

        void Start()
        {
            magmo = FindObjectOfType<MagneMotion>();
            if (magmo==null)
            {
                Debug.LogError($"MmRobot counld not find object of type MagneMotion");
            }
            var vacGripperName = "world/base_link/link1/link2/link3/link4/link5/link6/tool_link/gripper_base/Visuals/unnamed/RS007_Gripper_C_u";
            vgriptrans = transform.Find(vacGripperName);
            AddBoxToRobot(vgriptrans);
        }

        public void AddBoxToRobot(Transform vgriptrans)
        {
            if (vgriptrans == null)
            {
                Debug.LogError("AddBoxToRobot - Robot is null");
                return;
            }
            var prefab = Resources.Load<GameObject>("Prefabs/Box1");
            boxgo = Object.Instantiate<GameObject>(prefab);
            boxgo.name = "RobBox";
            var ska = 1f;
            boxgo.transform.localScale = new Vector3(ska, ska, ska);
            boxgo.transform.localRotation = Quaternion.Euler(0, 0, 0);
            boxgo.transform.localPosition = new Vector3(0, -0.14f, 0);
            boxgo.transform.SetParent(vgriptrans, worldPositionStays: false);
            ActivateRobBox(false);
        }
        float lasttimeset = -99;
        float lockpause = 0.01f;
        public bool ActivateRobBox(bool newstat)
        {
            var rv = false;
            if (boxgo != null)
            {
                if ((Time.time - lasttimeset) > lockpause)
                {
                    rv = boxgo.activeSelf;
                    boxgo.SetActive(newstat);
                    loadState = newstat;
                    lasttimeset = Time.time;
                }
            }
            return rv;
        }

        public void TransferBox()
        {
            switch (magmo.mmMode)
            {
                case MmMode.SimulateRailToRail:
                    {
                        var s = magmo.mmt.FindStoppedSled(!loadState);
                        if (s == null)
                        {
                            Debug.LogError($"MmRobot.TransferBox - cound not find stoppedsled with loadState {!loadState}");
                            return;
                        }
                        s.SetLoadState(loadState);
                        ActivateRobBox(!loadState);
                        break;
                    }
            }
        }

        void Update()
        {
            if (transferBox)
            {
                TransferBox();
                transferBox = !transferBox;
            }
        }
    }
}