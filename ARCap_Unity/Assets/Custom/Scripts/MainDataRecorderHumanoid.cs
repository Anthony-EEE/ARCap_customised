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


public class MainDataRecorderHumanoid : MonoBehaviour
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
    // private Vector3 right_pos_offset = new Vector3(0.1f, -0.02f, -0.07f); //+/-: [down/up, right/left, front/back]
    // private Vector3 left_pos_offset = new Vector3(-0.1f, -0.04f, -0.07f);
    private Vector3 right_pos_offset = new Vector3(0.0f, -0.013f, 0.095f);
    private Vector3 left_pos_offset = new Vector3(0.0f, -0.007f, 0.095f);

    private Quaternion right_orn_offset = Quaternion.Euler(0, 0, -90);
    private Quaternion left_orn_offset = Quaternion.Euler(0, 0, 90);

    // private GameObject rrobot;
    // private GameObject rgripper;
    // private GameObject rrobot_ee;

    // private GameObject lrobot;
    // private GameObject lgripper;
    // private GameObject lrobot_ee;

    private string folder_path;
    private float current_time = 0.0f;
    // Some control flags
    private bool startRecording = false;
    private bool startRemoving = false;
    private Image image_r;
    private Image image_l;
    private Image image_u;
    private Image image_b;
    private OVRHand l_hand;
    private OVRSkeleton l_hand_skeleton;
    private OVRHand r_hand;
    private OVRSkeleton r_hand_skeleton;
    private float[] quest_hand;
    private GameObject l_leap;
    private GameObject r_leap;
    private GameObject g1_l_wrist;
    private GameObject g1_r_wrist;
    private bool virutalApressed = false;
    private bool virtualAup = false;
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
        // rgripper.GetComponent<ArticulationBody>().TeleportRoot(rrobot_ee.transform.position + rrobot_ee.transform.rotation * rgripper_rot_offset * rgripper_pos_offset, rrobot_ee.transform.rotation * rgripper_rot_offset);
        // lgripper.GetComponent<ArticulationBody>().TeleportRoot(lrobot_ee.transform.position + lrobot_ee.transform.rotation * lgripper_rot_offset * lgripper_pos_offset, lrobot_ee.transform.rotation * lgripper_rot_offset);
        
        // Should use left eye anchor pose from OVRCameraRig
        var headPose = cameraRig.centerEyeAnchor.position;
        var headRot = cameraRig.centerEyeAnchor.rotation; // Should store them separately. [w,x,y,z]
        m_TimeText.enabled = true;
        checkVirtualAup();
        // Left controller on left hand
        Vector3 leftWristPos = cameraRig.leftHandAnchor.position;
        Vector3 rightWristPos = cameraRig.rightHandAnchor.position;
        Quaternion leftWristRot = cameraRig.leftHandAnchor.rotation;
        Quaternion rightWristRot = cameraRig.rightHandAnchor.rotation;
        l_leap.transform.position = g1_l_wrist.transform.position + g1_l_wrist.transform.rotation * left_pos_offset;
        l_leap.transform.rotation = g1_l_wrist.transform.rotation * left_orn_offset;
        r_leap.transform.position = g1_r_wrist.transform.position + g1_r_wrist.transform.rotation * right_pos_offset;
        r_leap.transform.rotation = g1_r_wrist.transform.rotation * right_orn_offset;
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
        computeLocalFingertip();
        string message = record + "BHand:" + left_wrist_pos.x + "," + left_wrist_pos.y + "," + left_wrist_pos.z + "," + left_wrist_rot.x + "," + left_wrist_rot.y + "," + left_wrist_rot.z + "," + left_wrist_rot.w;
        message = message + "," + right_wrist_pos.x + "," + right_wrist_pos.y + "," + right_wrist_pos.z + "," + right_wrist_rot.x + "," + right_wrist_rot.y + "," + right_wrist_rot.z + "," + right_wrist_rot.w;
        message = message + "," + head_pos.x + "," + head_pos.y + "," + head_pos.z + "," + head_orn.x + "," + head_orn.y + "," + head_orn.z + "," + head_orn.w;
        for(int i=0;i<18;i++)
        {
            message = message + "," + quest_hand[i];
        }
        byte[] data = Encoding.UTF8.GetBytes(message);
        sender.SendTo(data, data.Length, SocketFlags.None, targetEndPoint);
    }   
    // Has to be called after updateVisSpheres

    private void updateVisSpheres()
    {
        spheres[0].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        spheres[1].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        spheres[2].transform.position = (l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position+l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position+l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position)/3.0f;
        spheres[3].transform.position = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.position;
        spheres[4].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        spheres[5].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        spheres[6].transform.position = (r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position+r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position+r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position)/3.0f;
        spheres[7].transform.position = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.position;
    }

    private void checkVirtualAup()
    {
        Vector3 little_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position;
        Vector3 thumb_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        Vector3 index_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        float dist = Vector3.Distance(little_tip, thumb_tip);
        float dist2 = Vector3.Distance(index_tip, thumb_tip);
        if (dist < 0.05 && dist2 > 0.05)
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

    private void computeLocalFingertip()
    {
        Vector3 l_root = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.position;
        Quaternion l_root_rot = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.rotation;
        Vector3 l_thumb_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        Vector3 l_index_tip = l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        Vector3 l_rest_tip = (l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position+l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position+l_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position)/3.0f;
        Vector3 l_local_thumb = Quaternion.Inverse(l_root_rot) * (l_thumb_tip - l_root);
        Vector3 l_local_index = Quaternion.Inverse(l_root_rot) * (l_index_tip - l_root);
        Vector3 l_local_rest = Quaternion.Inverse(l_root_rot) * (l_rest_tip - l_root);
        Vector3 r_root = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.position;
        Quaternion r_root_rot = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.rotation;
        Vector3 r_thumb_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.position;
        Vector3 r_index_tip = r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.position;
        Vector3 r_rest_tip = (r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_MiddleTip].Transform.position+r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_RingTip].Transform.position+r_hand_skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_PinkyTip].Transform.position)/3.0f;
        Vector3 r_local_thumb = Quaternion.Inverse(r_root_rot) * (r_thumb_tip - r_root);
        Vector3 r_local_index = Quaternion.Inverse(r_root_rot) * (r_index_tip - r_root);
        Vector3 r_local_rest = Quaternion.Inverse(r_root_rot) * (r_rest_tip - r_root);
        quest_hand = new float[]{l_local_thumb.x, l_local_thumb.y, l_local_thumb.z, l_local_index.x, l_local_index.y, l_local_index.z, l_local_rest.x, l_local_rest.y, l_local_rest.z, r_local_thumb.x, r_local_thumb.y, r_local_thumb.z, r_local_index.x, r_local_index.y, r_local_index.z, r_local_rest.x, r_local_rest.y, r_local_rest.z};
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

        // Find robot
        l_leap = GameObject.Find("left_palm");
        r_leap = GameObject.Find("right_palm");
        g1_l_wrist = GameObject.Find("left_wrist_yaw_link_vis");
        g1_r_wrist = GameObject.Find("right_wrist_yaw_link_vis");

        // Set depth map index 200 x 200, from 0 to 1 in x and y
        // for (int y = 0; y < 100; y++){
        //     for (int x = 0; x < 100; x++)
        //     {
        //         depth_coords.Add(new Vector2(x / 100f, y / 100f));
        //     }
        // }

        if (!TryGetComponents())
        {
            m_TimeText.text = "missing components";
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
        folder_path = CoordinateFrameG1.folder_path;
        // Create sender socket
        sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        targetEndPoint = new IPEndPoint(IPAddress.Parse(CoordinateFrameG1.remote_ip), sender_port);
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