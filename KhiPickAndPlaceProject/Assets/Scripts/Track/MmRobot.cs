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
        public MmBox box;
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
            boxgo = box.gameObject;
            boxgo.transform.parent = null;
            boxgo.transform.localRotation = Quaternion.Euler(90, 0, 0);
            boxgo.transform.localPosition = new Vector3(0, -0.14f, 0);
            boxgo.transform.SetParent(vgriptrans, worldPositionStays: false);
            loadState = true;
            box.SetBoxStatus(BoxStatus.onRobot);
        }
        public MmBox DetachhBoxFromRobot()
        {
            var rv = box;
            box = null;
            boxgo = null;
            loadState = false;
            return rv;
        }
        //float lasttimeset = -99;
        //float lockpause = 0.01f;
        //public bool ActivateRobBoxOld(bool newstat)
        //{
        //    var rv = false;
        //    if (boxgo != null)
        //    {
        //        if ((Time.time - lasttimeset) > lockpause)
        //        {
        //            rv = boxgo.activeSelf;
        //            boxgo.SetActive(newstat);
        //            loadState = newstat;
        //            lasttimeset = Time.time;
        //        }
        //    }
        //    return rv;
        //}

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
            if (boxgo != null)
            {
                rv = boxgo.activeSelf;
                boxgo.SetActive(newstat);
                loadState = newstat;
            }
            return rv;
        }
        void Update()
        {

        }
    }
}