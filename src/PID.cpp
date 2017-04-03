#include <PID.h>

PID::PID()
{
	//Topic you want to publish
    // pub_gyro_ = n_.advertise<geometry_msgs::Vector3>("gyro_good", 10);
    // pub_acc_ = n_.advertise<geometry_msgs::Vector3>("acc_good", 10);
    // pub_q_est_= n_.advertise<geometry_msgs::Quaternion>("q_est", 10);
    // pub_euler_ = n_.advertise<geometry_msgs::Vector3>("RPY", 10);
    pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_class/cube_ref", 10);
    pub_to_matlab = n_.advertise<geometry_msgs::Vector3>("corr_enc_rif", 10);

	//Topic you want to subscribe
    sub_gyro_= n_.subscribe("/gyro_good", 10, &PID::callback_gyro, this);
    sub_euler_ = n_.subscribe("/RPY", 10, &PID::callback_imu_euler, this);
    sub_gain_ = n_.subscribe("/gain_PID", 10, &PID::callback_gain_PID, this);
    sub_corr = n_.subscribe("/qb_class/cube_current", 10, &PID::callback_corr, this);
    sub_enc = n_.subscribe("/qb_class/cube_measurement", 10, &PID::callback_meas, this);





    flag_run1_ = flag_run2_ = flag_run3_ =  false;
    
    kp_ = kd_ = ki_ = 0.0;

    // gyro_ = Eigen::Vector3d::Zero();
    // euler_ = Eigen::Vector3d::Zero();
    

	

}

PID::~PID()
{

}


//tutte le acc provenienti dalle imu (NON MYO)
void PID::callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	gyro_ << msg->x, msg->y, msg->z;
  	flag_run1_ = true;

}

void PID::callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	euler_ << msg->x, msg->y, msg->z;
  	flag_run2_ = true;

}

void PID::callback_gain_PID(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	kp_ = msg->x;
  	ki_ = msg->y;  	
  	kd_ = msg->z;
}

void PID::callback_corr(const qb_interface::cubeCurrent::ConstPtr& msg)
{
	to_matlab.x = msg->current_m1[0];
}

void PID::callback_meas(const qb_interface::cubePos::ConstPtr& msg)
{
	// to_matlab.y = msg->p_1[0];
	enc1_ = msg->p_1[0];
	enc2_ = msg->p_2[0];
	
	flag_run3_ = true;
	
}






void PID::run()
{
	double th = 30.0;
	double offset_pitch =  0.0;//0.0375;
	qb_interface::cubeRef comm_pub_;
	double enc1_g;


	

	if(flag_run1_ && flag_run2_ && flag_run3_)
	{


		enc1_ = enc1_ * 4.0 * PI / 65536;
		enc2_ = enc2_ * 4.0 * PI / 65536;

		

		P = kp_ * (euler_(1) - offset_pitch)*(euler_(1) - offset_pitch)*sgn(euler_(1));
		if((P * sgn(P)) <= th)
		{
			P = th * sgn(P);
		}

		I += ki_ *(euler_(1) - offset_pitch) / 200;
		D = kd_ * gyro_(1);

		PD = P + I + D;



		
		

		if(PD < -100)
		{
			PD = -100;
		}
		
		if (PD > 100)
		{
			PD = 100;
		}
		// std::cout << "PD4:" << PD << std::endl;
		 

		

		// std::cout<<"PD"<< PD<< std::endl;
		comm_pub_.p_1.push_back(PD);
		comm_pub_.p_2.push_back(-PD);

		pub_comm_.publish(comm_pub_);
		// std::cout<<"P:"<< P<<" "<<"I:"<< I<< " "<<"D:"<< D<< std::endl;
		std::cout<<"enc1_:"<< enc1_g<< std::endl;

		// to_matlab.z = PD;
		// pub_to_matlab.publish(to_matlab);



	}
	





}

int PID::sgn(double d){
    return d<0? -1 : d>0; 
    }




