#include "callback.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 추가
#include <algorithm>
#include <cmath>  
#include <iomanip>
#include "robot_msgs/msg/line_result.hpp"
#include "sensor_msgs/msg/imu.hpp" //imu sensor
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

bool flgflg = 0;
FILE *Trajectory_all;

Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr)
    : Node("callback_node_call"),  // Node 생성 시 노드 이름을 추가
      trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      pick_Ptr(pick_Ptr),
      SPIN_RATE(100)     
{
    // ROS 2에서의 Node 객체 생성
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("callback_node");
    
    // ROS 2에서 boost::thread 대신 std::thread 사용
    std::thread queue_thread(&Callback::callbackThread, this);
    queue_thread.detach();  // 비동기식 실행

    // Direction_subsciption_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu/data", 10, std::bind(&Callback::DirectionCallback, this, std::placeholders::_1));


    // set_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/SET", 10, std::bind(&Callback::SetMode, this, std::placeholders::_1));

    // ROS 2의 subscription 생성
    start_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/START", 10, std::bind(&Callback::StartMode, this, std::placeholders::_1));


    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
    omega_w = sqrt(g / z_c);

    pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 675);
    pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 675);

    
    emergency = 1;
    indext = 1;
    go = 0;

    RCLCPP_INFO(this->get_logger(), "Emergency value: %d", emergency);
    RCLCPP_INFO(this->get_logger(), "Callback activated");
}


void Callback::Set()
{
    All_Theta[0] = 0.0;
    All_Theta[1] = -0.050419;
    All_Theta[2] = -0.785155;
    All_Theta[3] = -0.327585;
    All_Theta[4] = 0.959987;
    All_Theta[5] = -0.032966;
    All_Theta[6] = 0.0;
    All_Theta[7] = 0.036848;
    All_Theta[8] = 0.785155;
    All_Theta[9] = 0.327585;
    All_Theta[10] = -0.907627;
    All_Theta[11] = -0.032966;

    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] - 24 * DEG2RAD; // neck_UD
}


int Callback::GetTurnsRemaining() const {
    return turns_remaining_.load(std::memory_order_acquire);
}
const void* Callback::GetTurnsRemainingAddr() const {
    return static_cast<const void*>(&turns_remaining_);
}
// "남은 유닛 턴 수"를 delta만큼 감소시키고, '감소 전 값'을 반환.
// acq_rel: 감소 전 읽기(acquire)와 감소 쓰기(release) 모두의 메모리 순서 보장.
int Callback::FetchSubTurnsRemaining(int delta) {
    return turns_remaining_.fetch_sub(delta, std::memory_order_acq_rel);
}

// "남은 유닛 턴 수"를 v로 설정. release 오더: 이후 acquire 로드에서 이 쓰기 이전의 효과가 보장된다.
void Callback::SetTurnsRemaining(int v) {
    turns_remaining_.store(v, std::memory_order_release);
}


void Callback::OnLineResult(int angle)
{
    
    if (!line_turn)
    {
        RCLCPP_INFO(this->get_logger(), "[OnLineResult] ignored because line_turn==false");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[OnLineResult] recv angle=%.2d, line_turn=%d",
            angle, (int)line_turn);

    double line_angle_ = angle;
    // 원래는 8도인데 실제 로봇이 8도를 넘게 돌아서 13으로 임의 설정
    const int stepdeg = 11;

    int turncount = static_cast<int>(line_angle_ / stepdeg);
    double extra_angle = static_cast<int>(line_angle_ - stepdeg * turncount);

    if (extra_angle > 9.5)
    {
        turncount += 1;
    }

    int before = turns_remaining_.load(std::memory_order_seq_cst);
    turns_remaining_.store(turncount, std::memory_order_seq_cst);
    int after  = turns_remaining_.load(std::memory_order_seq_cst);

    // RCLCPP_INFO(this->get_logger(),
    //   "[OnLineResult] turns_remaining_: %d -> %d (set=%d) @%p",
    //   before, after, turncount, static_cast<const void*>(&turns_remaining_));

    line_turn = false;

}


// // 현재 방향 기록
// void Callback::DirectionCallback(const sensor_msgs::msg::Imu::SharedPtr Direction_msg)
// {
//     tf2::Quaternion q(
//         Direction_msg->orientation.x,
//         Direction_msg->orientation.y,
//         Direction_msg->orientation.z,
//         Direction_msg->orientation.w);

//         // 쿼터니언 → roll, pitch, yaw 변환
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

//     yaw_now = yaw * 180.0 / M_PI; //라디안에서 dgree로
//     if (yaw_now < 0) yaw_now += 360; //0~360 보정
    
//     // 초기 방향 잡기 위해 한번만
//     if (!initial_yaw && yaw_now != 0){
//         initial_N = yaw_now; //북
//         //fmod(a,b) = a를 b로 나눈 나머지
//         initial_E = fmod(initial_N + 270, 360);   // 동
//         initial_W = fmod(initial_N + 90, 360);  // 서
//         initial_S = fmod(initial_N + 180, 360);  // 남
//         initial_yaw = true;
//         RCLCPP_INFO(this->get_logger(), "initial_N = %f, initial_E = %f, initial_W = %f, initial_S = %f",
//                 initial_N, initial_E, initial_W, initial_S);
//     }
// }

// 공 줍고 던진 후 몇도 돌아야할지 계산
// void Callback::CalculateTurn(int a)
// {
//     RCLCPP_INFO(this->get_logger(), "a = %d", a);
//     if(a == 1)
//     {
//         if(initial_E < yaw_now)
//         {
//             turn_angle1 = yaw_now - initial_E;
//         }
//         else if (initial_E > yaw_now)
//         {
//             turn_angle1 = yaw_now + 360 - initial_E;
//         }
//     }

//     else if(a == 2)
//     {
//         if(initial_S < yaw_now)
//         {
//             turn_angle1 = yaw_now - initial_S;
//         }
//         else if (initial_S > yaw_now)
//         {
//             turn_angle1 = yaw_now + 360 - initial_S;
//         }
//     }

//     else if(a == 3)
//     {
//         if(initial_E < yaw_now)
//         {
//             turn_angle1 = initial_E + 360 - yaw_now;
//         }
//         else if (initial_E > yaw_now)
//         {
//             turn_angle1 = initial_E - yaw_now;
//         }
//     }

//     else if(a == 4)
//     {
//         if(initial_N > yaw_now)
//         {
//             turn_angle1 = initial_N - yaw_now;
//         }
//         else if (initial_N < yaw_now)
//         {
//             turn_angle1 = initial_N + 360 - yaw_now;
//         }
//     }

//     angle111 = turn_angle1;
//     RCLCPP_INFO(this->get_logger(), "yaw_now = %f, turn_angle1 = %d", yaw_now, turn_angle1);

// }


// ros2 topic pub /START std_msgs/msg/Bool "data: true" -1

void Callback::StartMode(const std_msgs::msg::Bool::SharedPtr start)
{
    RCLCPP_DEBUG(this->get_logger(), "StartMode called with data: %d", start->data);
    if (start->data)
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);


        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);

        emergency = 0;
        // go = 1;

        RCLCPP_INFO(this->get_logger(), "StartMode activated with true data!");
    }
}


void Callback::TATA()
{
    double res_turn_angle = angle;

    if (res_turn_angle != 0)
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        // index_angle = 0;
    }
}


void Callback::callbackThread()
{
    // ROS 2의 spin 사용 대신 루프에서 메시지 처리
    rclcpp::Rate loop_rate(SPIN_RATE);
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

void Callback::SetLineTurn(bool on)
{
    line_turn.store(on, std::memory_order_release);
    RCLCPP_INFO(this->get_logger(), "[line_turn] <- %d", static_cast<int>(on));
}

void Callback::SelectMotion(int go)
{

    go_ = go;

    // RCLCPP_WARN(this->get_logger(), "[SelectMotion] called: go=%d, re=%d", go, re);
    if(re == 0)
    {
        if(go == 0) //start_pose
        {
            startRL_st[0] = 0;
            startRL_st[1] = 0.001941;
            startRL_st[2] = 0.122416;
            startRL_st[3] = 0.196013;
            startRL_st[4] = -0.073595;//1.148133;
            startRL_st[5] = 0.001941;

            startLL_st[0] = 0;
            startLL_st[1] = 0.001941;
            startLL_st[2] = -0.122416;
            startLL_st[3] = -0.196013;
            startLL_st[4] = 0.073595;//-1.148133;
            startLL_st[5] = 0.001941;

        }
        if(go == 1)// 6step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.3, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        
        }

        else if(go == 2)//좌회전
        {
            re = 1;
            indext = 0;
            angle = 8;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 
            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(1, 2, 0, -2, 1, 2, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 3)//우회전
        {
            re = 1;
            indext = 0;
            angle = 8;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(1, 2, 0, -2, 0, 1, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 4)//Back_2step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_2step;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.18, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4); 
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 5  || go == 22 || go == 28)//Back_Halfstep
        {
            re = 1;
            indext = 0;
            angle = 5;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.1, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }
                
        else if(go == 6)//Forward_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Forward_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight(0.01, 0.03, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(1, 1, 0, -1, 1, 1, 0, -3);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 7)//Left_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Left_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Left1(0.0275);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(-1, -1, -2, -6, 5, 7, -4, 0); //2,4,-3,1,2,7,2,-8
            IK_Ptr->Change_Angle_Compensation(2, 4, 2, 1, 2, 7, 0, -8);
            IK_Ptr->Set_Angle_Compensation(135);
        }


        else if(go == 8)//Right_Halfstep
        {
             re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Right_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Right1(0.0275);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(3, 7, 7, -5, 1, 1, 0.5, -5);
            // IK_Ptr->Change_Angle_Compensation(2, 7, 4, -8, 2, 4, -4, 4);

            // IK_Ptr->Change_Angle_Compensation(3, 7, 0, -5, 1, 1, 1.3, -4);
            IK_Ptr->Set_Angle_Compensation(135);
        }



        else if(go == 9)//PickMotion
        {
            re = 1;
            indext = 0;
            // mode = Motion_Index::Picking_Ball;
            
            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr -> Picking_Motion(300, 150, 0.165);
        }
        
        else if(go == 10)   //공 다시 잡기
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1,200);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 200);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 200);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 200);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 200);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 200);

            pick_Ptr->WT_Trajectory(0,200);
            pick_Ptr->RA_Trajectory(0,0,0,0,200);
            pick_Ptr->LA_Trajectory(-70,0,0,0,200);
            pick_Ptr->NC_Trajectory(0,30,200); // 공 다시 잡으면서 고개도 올리기
            // pick_Ptr->NC_Trajectory(0,0,250);
        }

        else if(go == 11) //Catch_Finish
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 150);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 150);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 150);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 150);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 150);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 150);

            pick_Ptr->WT_Trajectory(0,150);
            pick_Ptr->RA_Trajectory(0,0,0,0,150);  
            pick_Ptr->LA_Trajectory(70,0,0,30,150);
            pick_Ptr->NC_Trajectory(0,0,150);
        }

        else if(go == 12) //1 step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.15, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1.5, 2, 1, 0, -2.5); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 13) //forward_half_six
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight(0.02, 0.12, 0.03);    // basic
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
            IK_Ptr->Change_Angle_Compensation(1, 1, 0, -1, 1, 1, 0, -3);    // RL, LL plus outside
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 14) //forward_short
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight(0.01, 0.08, 0.03);   // closed

            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
            IK_Ptr->Change_Angle_Compensation(1, 1, 0, -1, 1, 1, 0, -3);    // RL, LL plus outside
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if (go == 17) //shoot_ready
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(-10,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(-180,-22,50,-50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (go == 18) //shoot
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(10);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 400);
        }

        else if (go == 19) //shoot_finish
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            // trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            // trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);
            trajectoryPtr->Ref_LL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);

            pick_Ptr->WT_Trajectory(20,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(150,22,-110,50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,-30,trajectoryPtr->Ref_RL_x.cols()); // 고개 내리기
            // pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (go == 77) //recovery
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 1900);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 1900);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 1900);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 1900);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 1900);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 1900);
        }

        else if(go == 20)//Huddle
        {
            // re = 1;
            // indext = 0;
            // trajectoryPtr->Change_Freq(2);
            // // mode = Motion_Index::Huddle;
            // IK_Ptr->Change_Com_Height(35);
            // trajectoryPtr->Huddle_Motion(0.22, 0.14, 0.05);
            // IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // // IK_Ptr->Change_Angle_Compensation(6.5, 2.6, 0.0, 3.5, 6.5, 2.6, 0.0, -3.5);
            // IK_Ptr->Change_Angle_Compensation(6.5, 1.0, 0.0, 3.5, 6.5, 1.0, 0.0, 0.0);

            // IK_Ptr->Set_Angle_Compensation(135);
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 2700);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 2700);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 2700);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 2700);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 2700);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 2700);
        }

        else if(go == 21)//fast_forward_4step
        {


            re = 1;
            indext = 0;
            // angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 3, -1.5, 2, 1, -3, -2.5);   
            IK_Ptr->Set_Angle_Compensation(67);
        }

        else if (go == 23) //Up_Nc
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,30,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (go == 24) //Down_Nc
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,-30,trajectoryPtr->Ref_RL_x.cols());

        }

        else if(go == 25)//40step
        {
            re = 1;
            indext = 0;
            angle = 2.65;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 2.5, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 26)//15step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 1.5, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 27)//2step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 29) //forward_left
        {
            re = 1;
            indext = 0;
            angle = 10;
            // index_angle = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, 1, 2, 1, 0, -5); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 30) //forward_right
        {
            re = 1;
            indext = 0;
            angle = 10;
            // index_angle = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, 1, 2, 1, 0, -5); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 31) //Re_Catch2
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1,150);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 150);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 150);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 150);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 150);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 150);

            pick_Ptr->WT_Trajectory(0,150);
            pick_Ptr->RA_Trajectory(0,0,0,0,150);
            pick_Ptr->LA_Trajectory(0,0,0,-30,150);
            pick_Ptr->NC_Trajectory(0,0,150); 
        }

        else if(go == 32)//40step fast
        {
            re = 1;
            indext = 0;

#ifdef STEP_COMMAND32_START_BLEND_TEST
            command32_start_trajectory_frames_ = 0;
#endif
#ifdef STEP_COMMAND32_COMPENSATION_TEST
            command32_compensation_frame_ = 0;
#endif

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
#ifdef STEP_COMMAND32_START_BLEND_TEST
            // Generate a command_1-style start reference at command_32's
            // actual walking frequency, then transition into the original
            // long-distance cruise reference over the first walking cycle.
            trajectoryPtr->Change_Freq(1.5);
            trajectoryPtr->Go_Straight_start(0.05, 2.5, 0.05);
            const MatrixXd start_rl_x = trajectoryPtr->Ref_RL_x;
            const MatrixXd start_rl_y = trajectoryPtr->Ref_RL_y;
            const MatrixXd start_rl_z = trajectoryPtr->Ref_RL_z;
            const MatrixXd start_ll_x = trajectoryPtr->Ref_LL_x;
            const MatrixXd start_ll_y = trajectoryPtr->Ref_LL_y;
            const MatrixXd start_ll_z = trajectoryPtr->Ref_LL_z;
#endif
            trajectoryPtr->Freq_Change_Straight(0.05, 2.5, 0.05, 1.5);
#ifdef STEP_COMMAND32_START_BLEND_TEST
            command32_start_trajectory_frames_ = std::min(
                trajectoryPtr->Return_Walktime_n(),
                static_cast<int>(trajectoryPtr->Ref_RL_x.cols())
            );
            for (int frame = 0; frame < command32_start_trajectory_frames_;
                 ++frame)
            {
                const double normalized_time = static_cast<double>(frame)
                    / static_cast<double>(
                        command32_start_trajectory_frames_ - 1);
                const double cruise_factor = normalized_time * normalized_time
                    * (3.0 - 2.0 * normalized_time);
                const double start_factor = 1.0 - cruise_factor;

                trajectoryPtr->Ref_RL_x(0, frame) =
                    start_factor * start_rl_x(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_RL_x(0, frame);
                trajectoryPtr->Ref_RL_y(0, frame) =
                    start_factor * start_rl_y(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_RL_y(0, frame);
                trajectoryPtr->Ref_RL_z(0, frame) =
                    start_factor * start_rl_z(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_RL_z(0, frame);
                trajectoryPtr->Ref_LL_x(0, frame) =
                    start_factor * start_ll_x(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_LL_x(0, frame);
                trajectoryPtr->Ref_LL_y(0, frame) =
                    start_factor * start_ll_y(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_LL_y(0, frame);
                trajectoryPtr->Ref_LL_z(0, frame) =
                    start_factor * start_ll_z(0, frame)
                    + cruise_factor * trajectoryPtr->Ref_LL_z(0, frame);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "[COMMAND32_START_TRAJECTORY] applied start-to-cruise transition over %d frames",
                command32_start_trajectory_frames_
            );
#endif
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3);
            IK_Ptr->Set_Angle_Compensation(101);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(go == 33)//Step_In_Place
        {
            re = 1;
            indext = 0;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 
            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(1, 2, 0, -2, 1, 2, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }


    }

    
}

// 동작 중인지 판별
bool Callback::IsMotionFinish()
{
    return (indext >= trajectoryPtr->Ref_RL_x.cols());
}

void Callback::ResetMotion()
{
    indext = 0;
    re = 0;
    index_angle = 0;

#ifdef STEP_ROLL_RATE_LIMIT_SAFETY
    prev_safe_all_theta_.setZero();
    roll_rate_limit_initialized_ = false;
#endif

#ifdef STEP_ROLL_SCALE_TEST
    roll_scale_reference_.setZero();
    roll_scale_reference_initialized_ = false;
    roll_scale_reference_command_ = -1;
#endif

#ifdef STEP_COMMAND32_START_BLEND_TEST
    command32_start_trajectory_frames_ = 0;
#endif

#ifdef STEP_COMMAND32_COMPENSATION_TEST
    command32_compensation_frame_ = 0;
#endif

#ifdef STEP_SAFETY_COMMAND_LOG
    // Keep the command-log frame monotonic across motion resets so multiple
    // motions remain distinguishable in one CSV. Only the roll guard state
    // needs reinitialization here.
#endif

    RCLCPP_INFO(rclcpp::get_logger("Callback"), "[ResetMotion] 인덱스 및 상태 초기화 완료");
}

#ifdef STEP_SAFETY_COMMAND_LOG
void Callback::LogSafetyCommands(
    const VectorXd& raw_all_theta,
    const VectorXd& scaled_all_theta,
    double roll_scale_right,
    double roll_scale_left,
    bool roll_scale_applied,
    bool command32_start_blend,
    double command32_blend_factor,
    double command32_roll_blend_factor,
    double command32_pitch_blend_factor,
    double command32_right_pitch_blend_factor,
    double command32_left_pitch_blend_factor,
    double command32_roll_scale,
    bool command32_roll_scale_applied,
    bool command32_start_trajectory_test,
    int command32_start_trajectory_phase,
    bool command32_compensation_test,
    double command32_compensation_scale,
    const char* command32_compensation_phase,
    const std::array<bool, NUMBER_OF_DYNAMIXELS>& roll_guard_used,
    bool roll_guard_enabled
)
{
    constexpr const char* safety_command_log_path =
        "/home/yeon/IRC/IRC-STEP/Dynamics/safety_control/"
        "safety_all_theta_command_log.csv";

    // The dry-run verification script rotates this file between commands.
    // Check only when the motion command changes to avoid filesystem work in
    // every control-loop iteration.
    static int last_logged_go = -1;
    if (go_ != last_logged_go)
    {
        if (safety_command_log_.is_open())
        {
            std::ifstream log_file(safety_command_log_path);
            if (!log_file.good())
            {
                safety_command_log_.close();
                safety_command_log_initialized_ = false;
            }
        }
        last_logged_go = go_;
    }

    if (!safety_command_log_initialized_)
    {
        safety_command_log_.open(safety_command_log_path);
        if (!safety_command_log_.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open safety command log: %s",
                safety_command_log_path
            );
            safety_command_log_initialized_ = true;
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Opened safety command log: %s",
            safety_command_log_path
        );

        safety_command_log_ << "frame,go";
        safety_command_log_ << std::setprecision(17);
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",raw_" << joint_index;
        }
#ifdef STEP_ROLL_SCALE_TEST
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",scaled_" << joint_index;
        }
#endif
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",safe_" << joint_index;
        }
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",delta_" << joint_index;
        }
#ifdef STEP_ROLL_SCALE_TEST
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",scale_delta_" << joint_index;
        }
        for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
             ++joint_index)
        {
            safety_command_log_ << ",guard_delta_" << joint_index;
        }
        safety_command_log_
            << ",roll_scale_right"
            << ",roll_scale_left"
            << ",roll_scale_applied";
#endif
        safety_command_log_
            << ",command32_start_blend"
            << ",command32_blend_factor"
            << ",command32_roll_blend_factor"
            << ",command32_pitch_blend_factor"
            << ",command32_right_pitch_blend_factor"
            << ",command32_left_pitch_blend_factor"
            << ",command32_roll_scale"
            << ",command32_roll_scale_applied"
            << ",command32_start_trajectory_test"
            << ",command32_start_trajectory_phase"
            << ",command32_compensation_test"
            << ",command32_compensation_scale"
            << ",command32_compensation_phase"
            << ",roll_guard_enabled"
            << ",roll_guard_used_1"
            << ",roll_guard_used_5"
            << ",roll_guard_used_7"
            << ",roll_guard_used_11\n";
        safety_command_log_initialized_ = true;
    }

    if (!safety_command_log_.is_open())
    {
        return;
    }

    safety_command_log_ << safety_command_log_frame_++ << ',' << go_;
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ',' << raw_all_theta[joint_index];
    }
#ifdef STEP_ROLL_SCALE_TEST
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ',' << scaled_all_theta[joint_index];
    }
#endif
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ',' << All_Theta[joint_index];
    }
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ','
            << All_Theta[joint_index] - raw_all_theta[joint_index];
    }
#ifdef STEP_ROLL_SCALE_TEST
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ','
            << scaled_all_theta[joint_index] - raw_all_theta[joint_index];
    }
    for (int joint_index = 0; joint_index < NUMBER_OF_DYNAMIXELS;
         ++joint_index)
    {
        safety_command_log_ << ','
            << All_Theta[joint_index] - scaled_all_theta[joint_index];
    }
    safety_command_log_
        << ',' << roll_scale_right
        << ',' << roll_scale_left
        << ',' << (roll_scale_applied ? 1 : 0);
#else
    (void)scaled_all_theta;
    (void)roll_scale_right;
    (void)roll_scale_left;
    (void)roll_scale_applied;
#endif
    safety_command_log_
        << ',' << (command32_start_blend ? 1 : 0)
        << ',' << command32_blend_factor
        << ',' << command32_roll_blend_factor
        << ',' << command32_pitch_blend_factor
        << ',' << command32_right_pitch_blend_factor
        << ',' << command32_left_pitch_blend_factor
        << ',' << command32_roll_scale
        << ',' << (command32_roll_scale_applied ? 1 : 0)
        << ',' << (command32_start_trajectory_test ? 1 : 0)
        << ',' << command32_start_trajectory_phase
        << ',' << (command32_compensation_test ? 1 : 0)
        << ',' << command32_compensation_scale
        << ',' << command32_compensation_phase
        << ',' << (roll_guard_enabled ? 1 : 0)
        << ',' << (roll_guard_used[1] ? 1 : 0)
        << ',' << (roll_guard_used[5] ? 1 : 0)
        << ',' << (roll_guard_used[7] ? 1 : 0)
        << ',' << (roll_guard_used[11] ? 1 : 0)
        << '\n';
    safety_command_log_.flush();
}
#endif


void Callback::Write_All_Theta()
{
    bool command32_compensation_test = false;
    double command32_compensation_scale = 1.0;
    const char* command32_compensation_phase = "full";

    if (emergency == 0)
    {
        if(re == 1)
        {
            int go = go_;
            // RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] indext = %d / step_n = %ld / go = %d", indext, trajectoryPtr->Ref_RL_x.cols(), go);
            
            if (go == 21) //fast_forward 4step
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Fast_Angle_Compensation(indext);
                if(indext>=67 && indext<=134)
                {
                    IK_Ptr->RL_th[0] = (trajectoryPtr->Turn_Trajectory(index_angle));
                    step = (IK_Ptr->RL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << "walktime_n" << walktime_n << std::endl;
                    if (index_angle > 66)
                    {
                        index_angle = 0;
                    }
                }
                // IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 2 || go == 29)//Step_in_place 좌회전, forward_left
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // std::cout << "indext" << indext << std::endl;
                if(indext > 135 && indext <= 270)
                {

                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = (IK_Ptr->LL_th[0])/2;
                    index_angle = index_angle + 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                    }
                }
            }

            else if (go == 3 || go == 30)//Step_in_place 우회전, forward_right
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // std::cout << "indext" << indext << std::endl;
                if(indext>=67 && indext <=337)
                {
                    IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                    step = (IK_Ptr->RL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                    }
                }
            }

            else if (go == 4 || go == 5 || go == 22 || go == 28 || go == 33)//Back_Halfstep //Back_2step
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // pick_Ptr->Back_Halfstep(trajectoryPtr->Ref_RL_x, indext);
                // std::cout << "indext" << indext << std::endl;

                if(indext>=67)
                {
                    IK_Ptr->LL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                    step = (IK_Ptr->LL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                    }
                }
            }

            else if (go == 6)//Forward_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 7)//Left_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Leftwalk(indext);
            }

            else if (go == 8)//Right_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Rightwalk(indext);
            }
            
            else if (go == 9) // Pick
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                
                pick_Ptr->Picking(trajectoryPtr->Ref_RL_x, indext, RL_th2, LL_th2);
            }

            else if (go == 10 || go == 11 || go == 17 || go == 19 || go == 24 || go == 23 || go == 31) //Re_Catch, Catch_Finish, shoot_ready, shoot_finish, Up_Nc, Down_Nc
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);

            }
            else if (go == 12 || go == 1 || go == 25 || go ==26 || go == 27 || go == 13 || go == 14) // forward 모음
            {                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                
                if(go == 25)
                {

                    // std::cout << "indext" << indext << std::endl;
                    int period = 405;
                    int local_index = indext % period;

                    if (local_index >= 67 && local_index <= 337)
                    {
                        IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                        step = (IK_Ptr->RL_th[0]) / 2;
                        index_angle += 1;

                        if (index_angle > walktime_n - 1)
                        {
                            index_angle = 0;
                        }
                    }

                }
            }
            
            else if (go == 18) //shoot
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                pick_Ptr->WT_th[0] =  -20 * DEG2RAD;
                pick_Ptr->LA_th[0] = -150 * DEG2RAD;
                pick_Ptr->LA_th[1] = -22 * DEG2RAD;
                pick_Ptr->LA_th[2] = 110 * DEG2RAD;
                pick_Ptr->LA_th[3]  = -50 * DEG2RAD;
            }

            else if (go == 32) //forward 40step fast
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
#ifdef STEP_COMMAND32_COMPENSATION_TEST
                constexpr std::uint32_t COMMAND32_COMP_HOLD_FRAMES = 80;
                constexpr std::uint32_t COMMAND32_COMP_RAMP_FRAMES = 135;

                command32_compensation_test = true;
                if (command32_compensation_frame_
                    < COMMAND32_COMP_HOLD_FRAMES)
                {
                    command32_compensation_scale = 0.0;
                    command32_compensation_phase = "hold";
                }
                else if (command32_compensation_frame_
                    < COMMAND32_COMP_HOLD_FRAMES
                        + COMMAND32_COMP_RAMP_FRAMES)
                {
                    const std::uint32_t ramp_frame =
                        command32_compensation_frame_
                        - COMMAND32_COMP_HOLD_FRAMES;
                    const double t = static_cast<double>(ramp_frame)
                        / static_cast<double>(
                            COMMAND32_COMP_RAMP_FRAMES - 1);
                    command32_compensation_scale =
                        t * t * (3.0 - 2.0 * t);
                    command32_compensation_phase = "ramp";
                }
                else
                {
                    command32_compensation_scale = 1.0;
                    command32_compensation_phase = "full";
                }

                ++command32_compensation_frame_;

                double rl_before_compensation[6];
                double ll_before_compensation[6];
                for (int joint_index = 0; joint_index < 6; ++joint_index)
                {
                    rl_before_compensation[joint_index] =
                        IK_Ptr->RL_th[joint_index];
                    ll_before_compensation[joint_index] =
                        IK_Ptr->LL_th[joint_index];
                }

                IK_Ptr->Forward_40step_Angle_Compensation(indext);

                for (int joint_index = 0; joint_index < 6; ++joint_index)
                {
                    IK_Ptr->RL_th[joint_index] =
                        rl_before_compensation[joint_index]
                        + command32_compensation_scale
                            * (IK_Ptr->RL_th[joint_index]
                                - rl_before_compensation[joint_index]);
                    IK_Ptr->LL_th[joint_index] =
                        ll_before_compensation[joint_index]
                        + command32_compensation_scale
                            * (IK_Ptr->LL_th[joint_index]
                                - ll_before_compensation[joint_index]);
                }
#else
                IK_Ptr->Forward_40step_Angle_Compensation(indext);
#endif
            }

            else if (go == 77)
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                pick_Ptr->Stand_up(trajectoryPtr->Ref_RL_x, indext);
            }
            else if (go == 20)
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                
                pick_Ptr->hhhh(trajectoryPtr->Ref_RL_x, indext);
            }
        }

    }

    indext += 1;

    if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
    {

        re = 0;
    }



    // All_Theta 계산 및 저장
    All_Theta[0] = -IK_Ptr->RL_th[0] + pick_Ptr->RL_th_ALL[0] + startRL_st[0];
    All_Theta[1] = IK_Ptr->RL_th[1] + pick_Ptr->RL_th_ALL[1] +startRL_st[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] + pick_Ptr->RL_th_ALL[2] +startRL_st[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    All_Theta[3] = -IK_Ptr->RL_th[3] + pick_Ptr->RL_th_ALL[3] +startRL_st[3] + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    All_Theta[4] = -IK_Ptr->RL_th[4] + pick_Ptr->RL_th_ALL[4] +startRL_st[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + pick_Ptr->RL_th_ALL[5] +startRL_st[5] - 2* DEG2RAD;

    All_Theta[6] = -IK_Ptr->LL_th[0] + pick_Ptr->LL_th_ALL[0] +startLL_st[0];
    All_Theta[7] = IK_Ptr->LL_th[1] + pick_Ptr->LL_th_ALL[1] +startLL_st[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + pick_Ptr->LL_th_ALL[2] +startLL_st[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    All_Theta[9] = IK_Ptr->LL_th[3] + pick_Ptr->LL_th_ALL[3] +startLL_st[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    All_Theta[10] = IK_Ptr->LL_th[4] + pick_Ptr->LL_th_ALL[4] +startLL_st[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + pick_Ptr->LL_th_ALL[5] +startLL_st[5] - 2 * DEG2RAD;

    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] - 24 * DEG2RAD; // neck_UD

    bool command32_start_blend = false;
    double command32_blend_factor = 1.0;
    double command32_roll_blend_factor = 1.0;
    double command32_pitch_blend_factor = 1.0;
    double command32_right_pitch_blend_factor = 1.0;
    double command32_left_pitch_blend_factor = 1.0;
    double command32_roll_scale = 1.0;
    bool command32_roll_scale_applied = false;
    bool command32_start_trajectory_test = false;
    int command32_start_trajectory_phase = 0;

#ifdef STEP_COMMAND32_START_BLEND_TEST
    if (go_ == 32 && command32_start_trajectory_frames_ > 0)
    {
        command32_start_trajectory_test = true;
        // Phase 1: first-cycle start-to-cruise transition. Phase 2: original
        // Freq_Change_Straight long-distance trajectory.
        command32_start_trajectory_phase =
            indext <= command32_start_trajectory_frames_ ? 1 : 2;
    }
#endif

#if defined(STEP_SAFETY_COMMAND_LOG) || defined(STEP_ROLL_SCALE_TEST)
    const VectorXd raw_all_theta = All_Theta;
#endif

    double roll_scale_right = 1.0;
    double roll_scale_left = 1.0;
    bool roll_scale_applied = false;

#ifdef STEP_ROLL_SCALE_TEST
    if (go_ == 1)
    {
        roll_scale_right = 0.50;
        roll_scale_left = 0.50;
        roll_scale_applied = true;
    }

    if (!roll_scale_reference_initialized_
        || roll_scale_reference_command_ != go_)
    {
        roll_scale_reference_ = raw_all_theta;
        roll_scale_reference_initialized_ = true;
        roll_scale_reference_command_ = go_;
    }

    constexpr int right_roll_joint_indices[] = {1, 5};
    constexpr int left_roll_joint_indices[] = {7, 11};
    for (int joint_index : right_roll_joint_indices)
    {
        All_Theta[joint_index] = roll_scale_reference_[joint_index]
            + roll_scale_right
                * (raw_all_theta[joint_index]
                    - roll_scale_reference_[joint_index]);
    }
    for (int joint_index : left_roll_joint_indices)
    {
        All_Theta[joint_index] = roll_scale_reference_[joint_index]
            + roll_scale_left
                * (raw_all_theta[joint_index]
                    - roll_scale_reference_[joint_index]);
    }
#endif

#ifdef STEP_SAFETY_COMMAND_LOG
    const VectorXd scaled_all_theta = All_Theta;
    std::array<bool, NUMBER_OF_DYNAMIXELS> roll_guard_used{};
#endif

#ifdef STEP_ROLL_RATE_LIMIT_SAFETY
    constexpr double roll_rate_limit = 0.05;
    constexpr int roll_joint_indices[] = {1, 5, 7, 11};

    if (!roll_rate_limit_initialized_)
    {
        for (int joint_index : roll_joint_indices)
        {
            prev_safe_all_theta_[joint_index] = All_Theta[joint_index];
        }
        roll_rate_limit_initialized_ = true;
    }
    else
    {
        for (int joint_index : roll_joint_indices)
        {
            const double delta = All_Theta[joint_index]
                - prev_safe_all_theta_[joint_index];
            const double clamped_delta = std::clamp(
                delta,
                -roll_rate_limit,
                roll_rate_limit
            );
            All_Theta[joint_index] =
                prev_safe_all_theta_[joint_index] + clamped_delta;
#ifdef STEP_SAFETY_COMMAND_LOG
            roll_guard_used[joint_index] =
                std::abs(clamped_delta - delta) > 1.0e-12;
#endif
            prev_safe_all_theta_[joint_index] = All_Theta[joint_index];
        }
    }
#endif

#ifdef STEP_SAFETY_COMMAND_LOG
#ifdef STEP_ROLL_RATE_LIMIT_SAFETY
    constexpr bool roll_guard_enabled = true;
#else
    constexpr bool roll_guard_enabled = false;
#endif
    LogSafetyCommands(
        raw_all_theta,
        scaled_all_theta,
        roll_scale_right,
        roll_scale_left,
        roll_scale_applied,
        command32_start_blend,
        command32_blend_factor,
        command32_roll_blend_factor,
        command32_pitch_blend_factor,
        command32_right_pitch_blend_factor,
        command32_left_pitch_blend_factor,
        command32_roll_scale,
        command32_roll_scale_applied,
        command32_start_trajectory_test,
        command32_start_trajectory_phase,
        command32_compensation_test,
        command32_compensation_scale,
        command32_compensation_phase,
        roll_guard_used,
        roll_guard_enabled
    );
#endif

    if(go == 0)
    {
        for (int i = 0; i < 6; i++) 
        {
            startRL_st[i] = 0.0;
            startLL_st[i] = 0.0;
        }
    }

    // 디버깅 정보 출력
//     for (int i = 0; i < All_Theta.size(); ++i)
//     {
//         RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
//     }
}
