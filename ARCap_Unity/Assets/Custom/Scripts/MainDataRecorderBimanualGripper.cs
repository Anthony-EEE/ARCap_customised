using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using UnityEngine.Rendering;
using Unity.XR.Oculus;
using Meta.XR.Depth;
using TMPro;
using UnityEditor;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;


public class MainDataRecorderBimanualGripper : MonoBehaviour
{
    // #region Constants
    // private static readonly int VirtualDepthTextureID = Shader.PropertyToID("_CameraDepthTexture");
    // #endregion // Constants
    // Define some variables for program
    #region Private Variables
    OVRCameraRig cameraRig;
    private UdpClient client;
    private IPEndPoint remoteEndPoint;
    private Socket sender;
    private IPEndPoint targetEndPoint;
    private GameObject[] spheres = new GameObject[8];
    private Quaternion rotateZX = Quaternion.Euler(45f, 0f,90f);
    private Quaternion rotateZXinv = Quaternion.Euler(45f, 0f, -90f);
    private Quaternion rotateX = Quaternion.Euler(-20f, -25f,-25f);
    private Quaternion rotateXinv = Quaternion.Euler(-30f, 25f, 25f);
    // private Vector3 right_pos_offset = new Vector3(0.1f, -0.02f, -0.07f); //+/-: [down/up, right/left, front/back]
    // private Vector3 left_pos_offset = new Vector3(-0.1f, -0.04f, -0.07f);
    private Vector3 right_pos_offset = new Vector3(0f, 0f, 0f);
    private Vector3 left_pos_offset = new Vector3(0f, 0f, 0f);
    private Vector3 rgripper_pos_offset = new Vector3(0f, 0f, 0f);
    private Quaternion rgripper_rot_offset = Quaternion.Euler(0f, 0f, 0f);
    private Vector3 lgripper_pos_offset = new Vector3(0f, 0f, 0f);
    private Quaternion lgripper_rot_offset = Quaternion.Euler(0f, 0f, 0f);

    private GameObject rrobot;
    private GameObject rgripper;
    private GameObject rrobot_ee;

    private GameObject lrobot;
    private GameObject lgripper;
    private GameObject lrobot_ee;

    private string folder_path;
    private float current_time = 0.0f;
    // Some control flags
    private bool startRecording = false;
    private bool startRemoving = false;
    private bool deleted = false;
    private bool virutalApressed = false;
    private bool virtualAup = false;
    private Image image_r;
    private Image image_l;
    private Image image_u;
    private Image image_b;
    private OVRHand l_hand;
    private OVRSkeleton l_hand_skeleton;
    private OVRHand r_hand;
    private OVRSkeleton r_hand_skeleton;

    #endregion


    #region Unity Inspector Variables
    // [SerializeField]
    // [Tooltip("The RawImage where the virtual depth map will be displayed.")]
    // private RawImage m_virtualDepthImage;
    [SerializeField]
    [Tooltip("Time text")]
    private TextMeshProUGUI m_TimeText;
    public string local_ip;
    [SerializeField]
    public int listen_port = 65432;
    [SerializeField]
    public int sender_port = 12346;
    public static int traj_cnt = 0;

    #endregion // Unity Inspector Variables

    #region Private Methods
    /// <summary>
    /// Attempts to get any unassigned components.
    /// </summary>
    /// <returns>
    /// <c>true</c> if all components were satisfied; otherwise <c>false</c>.
    /// </returns>
    private bool TryGetComponents()
    {
        if (m_TimeText == null) { m_TimeText = GetComponent<TextMeshProUGUI>(); }
        return m_TimeText != null;
    }
    /// <summary>
    /// Attempts to show the depth textures.
    /// </summary>
    /// <returns>
    /// <c>true</c> if the textures were shown; otherwise <c>false</c>.
    /// </returns>
    private bool MainLoop()
    {
        // Attempt to get the global depth texture
        // This should be a image, get a image and send via redis?
        rgripper.GetComponent<ArticulationBody>().TeleportRoot(rrobot_ee.transform.position + rrobot_ee.transform.rotation * rgripper_rot_offset * rgripper_pos_offset, rrobot_ee.transform.rotation * rgripper_rot_offset);
        lgripper.GetComponent<ArticulationBody>().TeleportRoot(lrobot_ee.transform.position + lrobot_ee.transform.rotation * lgripper_rot_offset * lgripper_pos_offset, lrobot_ee.transform.rotation * lgripper_rot_offset);
        
        // Should use left eye anchor pose from OVRCameraRig
        var headPose = cameraRig.centerEyeAnchor.position;
        var headRot = cameraRig.centerEyeAnchor.rotation; // Should store them separately. [w,x,y,z]
        m_TimeText.enabled = true;
        checkVirtualAup();
        // Left controller on right hand, inversed
        Vector3 leftWristPos = cameraRig.leftHandAnchor.position + cameraRig.leftHandAnchor.rotation * left_pos_offset;
        Vector3 rightWristPos = cameraRig.rightHandAnchor.position + cameraRig.rightHandAnchor.rotation * right_pos_offset;
        Quaternion leftWristRot = cameraRig.leftHandAnchor.rotation; //* rotateXinv * rotateZXinv;
        Quaternion rightWristRot = cameraRig.rightHandAnchor.rotation;// * rotateX * rotateZX;
        updateVisSpheres();
        //updateVisSpheres(hand, leftWristPos, leftWristRot, rightWristPos, rightWristRot);
        // if time gap > 0.05 send hand pose
        if (Time.time - current_time > 0.02)
        {
            if(startRecording)
            {
                SendHeadWristPose("Y", leftWristPos, leftWristRot, rightWristPos, rightWristRot, headPose, headRot);
            }
            else
            {
                SendHeadWristPose("N", leftWristPos, leftWristRot, rightWristPos, rightWristRot, headPose, headRot);
            }
            current_time = Time.time;
        }
        if (startRecording)
        {
            m_TimeText.text = "Recording:" + traj_cnt;
            // RecordData(counter++, headPose, headRot, 
            //             leftWristPos, leftWristRot,
            //             rightWristPos, rightWristRot,
            //             hand, current_time);
        }
        else
        {
            m_TimeText.text = "Not recording";
        }
        return true;
    }

    // handedness: "L" or "R"
    // record: "Y" or "N"
    private void SendHeadWristPose(string record, Vector3 left_wrist_pos, Quaternion left_wrist_rot, Vector3 right_wrist_pos, Quaternion right_wrist_rot, Vector3 head_pos, Quaternion head_orn)
    {
        m_TimeText.text = "Before send";
        float is_left_pinch = checkPinch(true);
        m_TimeText.text = "Before right";
        float is_right_pinch = checkPinch(false);
        string message = record + "BHand:" + left_wrist_pos.x + "," + left_wrist_pos.y + "," + left_wrist_pos.z + "," + left_wrist_rot.x + "," + left_wrist_rot.y + "," + left_wrist_rot.z + "," + left_wrist_rot.w;
        message = message + "," + right_wrist_pos.x + "," + right_wrist_pos.y + "," + right_wrist_pos.z + "," + right_wrist_rot.x + "," + right_wrist_rot.y + "," + right_wrist_rot.z + "," + right_wrist_rot.w;
        message = message + "," + head_pos.x + "," + head_pos.y + "," + head_pos.z + "," + head_orn.x + "," + head_orn.y + "," + head_orn.z + "," + head_orn.w + "," + is_left_pinch + "," + is_right_pinch;
        m_TimeText.text = "Message encoded";
        byte[] data = Encoding.UTF8.GetBytes(message);
        sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
        m_TimeText.text = "After";
    }   
    // Has to be called after updateVisSpheres
    
    private float computeDistance(float[] pos1, float[] pos2)
    {
        return Mathf.Sqrt(Mathf.Pow(pos1[0] - pos2[0], 2) + Mathf.Pow(pos1[1] - pos2[1], 2) + Mathf.Pow(pos1[2] - pos2[2], 2));
    }

    private float checkPinch(bool is_left)
    {
        if (is_left)
        {
            Vector3 thumb_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
            Vector3 index_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
            Vector3 middle_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
            float dist1 = Vector3.Distance(thumb_tip, index_tip);
            float dist2 = Vector3.Distance(thumb_tip, middle_tip);
            return (dist1 + dist2)/2;
        }
        else
        {
            Vector3 thumb_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
            Vector3 index_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
            Vector3 middle_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
            float dist1 = Vector3.Distance(thumb_tip, index_tip);
            float dist2 = Vector3.Distance(thumb_tip, middle_tip);
            return (dist1 + dist2)/2;
        }
        
    }

    private void updateVisSpheres()
    {
        spheres[0].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        spheres[1].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        spheres[2].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
        spheres[3].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position;
        spheres[4].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        spheres[5].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        spheres[6].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position;
        spheres[7].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position;
    }

    private void checkVirtualAup()
    {
        Vector3 little_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position;
        Vector3 ring_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position;
        float dist = Vector3.Distance(little_tip, ring_tip);
        if (dist < 0.03)
        {
            virutalApressed = true;
            virtualAup = false;
        }
        else
        {
            if (virutalApressed)
            {
                virtualAup = true;
            }
            virutalApressed = false;
        }

    }

    private bool getVirtualAup()
    {
        if (virtualAup)
        {
            virtualAup = false;
            return true;
        }
        return false;
    }

    #endregion // Private Methods

    #region Unity Message Handlers
    /// <summary>
    /// Start is called before the first frame update
    /// </summary>
    protected void Start()
    {
        // Load SelectWorldScene
        local_ip = StartScene.local_ip;
        cameraRig = GameObject.Find("OVRCameraRig").GetComponent<OVRCameraRig>();
        l_hand = GameObject.Find("LeftControllerAnchor").GetComponent<OVRHand>();
        l_hand_skeleton = GameObject.Find("LeftControllerAnchor").GetComponent<OVRSkeleton>();
        r_hand = GameObject.Find("RightControllerAnchor").GetComponent<OVRHand>();
        r_hand_skeleton = GameObject.Find("RightControllerAnchor").GetComponent<OVRSkeleton>();
        // Set depth map index 200 x 200, from 0 to 1 in x and y
        // for (int y = 0; y < 100; y++){
        //     for (int x = 0; x < 100; x++)
        //     {
        //         depth_coords.Add(new Vector2(x / 100f, y / 100f));
        //     }
        // }

        if (!TryGetComponents())
        {
            m_TimeText.text = "mising components";
            enabled = false;
        }
        // set up socket
        
        try
        {
            m_TimeText.text = "Socket connecting...";
            client = new UdpClient();
            client.Client.Bind(new IPEndPoint(IPAddress.Parse(local_ip), listen_port));
            //client.Client.Bind(new IPEndPoint(IPAddress.Any, 65432));
            remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
            m_TimeText.text = "Socket connected";
        }
        catch (Exception e)
        {
            m_TimeText.text = "Socket error: " + e.Message;
        }
        // Create 8 spheres for visualization
        for (int i = 0; i < 8; i++)
        {
            GameObject sphere = GameObject.Find("Sphere"+i);
            sphere.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
            sphere.transform.position = new Vector3(i, 0, 0);
            spheres[i] = sphere;
        }
        // Create a folder with current time world frame is based on right arm
        folder_path = CoordinateFrame.folder_path;
        // Visualize coordinate frame pos
        rrobot = GameObject.Find("panda_link0_vis");
        rrobot_ee = GameObject.Find("panda_hand_vis");
        rgripper = GameObject.Find("gripper_base_vis_right");
        GameObject rframe = GameObject.Find("coordinate_vis");
        
        lrobot = GameObject.Find("panda_link0_vis_gripper");
        lrobot_ee = GameObject.Find("panda_hand_vis_gripper");
        lgripper = GameObject.Find("gripper_base_vis");
        GameObject lframe = GameObject.Find("coordinate_vis_gripper");


        rframe.transform.position = RightCoordinateFrameGripper.last_pos;
        rframe.transform.rotation = RightCoordinateFrameGripper.last_rot;
        rrobot.GetComponent<ArticulationBody>().TeleportRoot(RightCoordinateFrameGripper.last_pos, RightCoordinateFrameGripper.last_rot);
        
        lframe.transform.position = LeftCoordinateFrameGripper.last_pos;
        lframe.transform.rotation = LeftCoordinateFrameGripper.last_rot;
        lrobot.GetComponent<ArticulationBody>().TeleportRoot(LeftCoordinateFrameGripper.last_pos, LeftCoordinateFrameGripper.last_rot);
        // Create sender socket
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        targetEndPoint = new IPEndPoint(IPAddress.Parse(LeftCoordinateFrameGripper.remote_ip), sender_port);
        // Initialize hand 
        current_time = Time.time;
        image_u = GameObject.Find("panel_r").GetComponent<Image>();
        image_r = GameObject.Find("panel_u").GetComponent<Image>();
        image_l = GameObject.Find("panel_l").GetComponent<Image>();
        image_b = GameObject.Find("panel_b").GetComponent<Image>();
    }

    /// <summary>
    /// Update is called once per frame
    /// </summary>
    protected void Update()
    {
        // Attempt to show the render textures
        MainLoop();
        if  (getVirtualAup())
        {
            startRecording = !startRecording;
            if (startRecording)
            {
                deleted = false;
                startRemoving = false;
                var head_pose = cameraRig.centerEyeAnchor.position;
                var head_rot = cameraRig.centerEyeAnchor.rotation;
                string message = "Start:"+head_pose.x+","+head_pose.y+","+head_pose.z+","+head_rot.x+","+head_rot.y+","+head_rot.z+","+head_rot.w;
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
                traj_cnt ++;
                image_r.color = new Color32(188, 12, 13, 100);
                image_b.color = new Color32(188, 12, 13, 100);
                image_u.color = new Color32(188, 12, 13, 100);
                image_l.color = new Color32(188, 12, 13, 100);
            }
            else
            {
                image_r.color = new Color32(12, 188, 13, 100);
                image_b.color = new Color32(12, 188, 13, 100);
                image_u.color = new Color32(12, 188, 13, 100);
                image_l.color = new Color32(12, 188, 13, 100);
            }
        }
        // if(OVRInput.GetUp(OVRInput.RawButton.B))
        // {
        //     if(traj_cnt > 0 && !deleted)
        //     {
        //         startRemoving = true;
        //         traj_cnt --;
        //         deleted = true;
        //     }
        //     if(startRecording)
        //     {
        //         startRecording = false;
        //         image_r.color = new Color32(12, 188, 13, 100);
        //         image_b.color = new Color32(12, 188, 13, 100);
        //         image_u.color = new Color32(12, 188, 13, 100);
        //         image_l.color = new Color32(12, 188, 13, 100);
        //     }
        // }
        if (!startRecording)
        {
            if (startRemoving)
            {
                string message = "Remove";
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
            }
            else
            {
                string message = "Stop";
                byte[] data = Encoding.UTF8.GetBytes(message);
                sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
            }
        }
        
    }

    protected void OnApplicationQuit()
    {
        if (client != null)
        {
            client.Close();
        }
    }
    #endregion // Unity Message Handlers
}