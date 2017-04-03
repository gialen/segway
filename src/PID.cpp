#include <PID.h>

PID::PID()
{
	//Topic you want to publish
    // pub_gyro_ = n_.advertise<geometry_msgs::Vector3>("gyro_good", 10);
    // pub_acc_ = n_.advertise<geometry_msgs::Vector3>("acc_good", 10);
    // pub_q_est_= n_.advertise<geometry_msgs::Quaternion>("q_est", 10);
    // pub_euler_ = n_.advertise<geometry_msgs::Vector3>("RPY", 10);
    pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_class/cube_ref", 10);
    // pub_to_matlab = n_.advertise<geometry_msgs::Vector3>("corr_enc_rif", 10);
    pub_vel_ = n_.advertise<geometry_msgs::Vector3>("vel", 10);

	//Topic you want to subscribe
    sub_gyro_= n_.subscribe("/gyro_good", 10, &PID::callback_gyro, this);
    sub_euler_ = n_.subscribe("/RPY", 10, &PID::callback_imu_euler, this);
    sub_gain_ = n_.subscribe("/gain_PID", 10, &PID::callback_gain_PID, this);
    sub_corr = n_.subscribe("/qb_class/cube_current", 10, &PID::callback_corr, this);
    sub_enc = n_.subscribe("/qb_class/cube_measurement", 10, &PID::callback_meas, this);
    sub_gain_v_ = n_.subscribe("/gain_PID_v", 10, &PID::callback_gain_PID_v, this);





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

void PID::callback_gain_PID_v(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	kp_v_ = msg->x;
  	ki_v_ = msg->y;  	
  	kd_v_ = msg->z;
}

void PID::callback_meas(const qb_interface::cubePos::ConstPtr& msg)
{
	// to_matlab.y = msg->p_1[0];
	if(!flag_run3_)
	{
		enc1_ = -msg->p_1[0] * 4.0 * PI / 65536;
		enc2_ = -msg->p_L[0] * 4.0 * PI / 65536;
		enc1_of_ = enc1_;
		enc2_of_ = enc2_;
		// enc1_old_ = enc1_;
		// enc2_old_ = enc2_;
		enc1_ = 0;
		enc2_ = 0;
		
		flag_run3_ = true;
	}
	enc1_ = -msg->p_1[0] * 4.0 * PI / 65536  - enc1_of_;
	enc2_ = -msg->p_L[0] * 4.0 * PI / 65536  - enc2_of_;
	
	
	
}






void PID::run()
{
	double th = 30.0;
	double offset_pitch =  0.0;//0.0375;
	qb_interface::cubeRef comm_pub_;
	double enc1_g, enc2_g, vel1_enc, vel2_enc, vel1, vel2, vel, pos1, pos2, pos, a;
	geometry_msgs::Vector3 vel_pub;

	double PI_v_der, vel_der;

	a = dt / (0.1 + dt);

	

	if(flag_run1_ && flag_run2_ && flag_run3_)
	{

		// enc1_ = enc1_ * 4.0 * PI / 65536;
		// enc2_ = enc2_ * 4.0 * PI / 65536;

		enc1_g = unwrap(enc1_old_, enc1_);
		enc2_g = unwrap(enc2_old_, enc2_);

		vel1_enc = (enc1_g - enc1_old_)/dt;
		vel2_enc = (enc2_g - enc2_old_)/dt;

		// vel1_enc = (1 - a) * vel1_old_ + (a * vel1_enc);
		// vel2_enc = (1 - a) * vel2_old_ + (a * vel2_enc);

		// vel1_old_ = vel1_enc;
		// vel2_old_ = vel2_enc;


		enc1_old_ = enc1_g;
		enc2_old_ = enc2_g;

		pos1 = euler_(1) + enc1_g;
		pos2 = euler_(1) - enc2_g;

		vel1 = gyro_(1) + vel1_enc;
		vel2 = gyro_(1) - vel2_enc;


		vel1 = (1 - a) * vel1_old_ + (a * vel1);
		vel2 = (1 - a) * vel2_old_ + (a * vel2);

		vel1_old_ = vel1;
		vel2_old_ = vel2;

		vel = (vel1 + vel2) / 2;
		pos = (pos1 + pos2) / 2;

		vel_der = (vel - vel_old_)/dt;
		vel_old_ = vel;

		//controllo doppio loop
		P_v = kp_v_ * (-vel);
		I_v = ki_v_ * (-pos);
		D_v = kd_v_ * (-vel_der);


		PI_v = P_v + I_v;

		PI_v_der = (PI_v - PI_v_old)/dt;
		PI_v_old = PI_v;

		P = kp_ * (PI_v-euler_(1));
		if((P * sgn(P)) <= th)
		{
			P = th * sgn(P);
		}

		I += ki_ *(PI_v-euler_(1)) / 200;


		D = kd_ * (PI_v_der-gyro_(1));

		Pid = P + I + D;




		
		// //solo loop interno
		// P = kp_ * (-euler_(1));
		// if((P * sgn(P)) <= th)
		// {
		// 	P = th * sgn(P);
		// }

		// I += ki_ *(-euler_(1)) / 200;
		// D = kd_ * (-gyro_(1));

		// Pid = P + I + D;



		
		

		if(Pid < -100)
		{
			Pid = -100;
		}
		
		if (Pid > 100)
		{
			Pid = 100;
		}
		// std::cout << "PD4:" << PD << std::endl;
		 

		

		// std::cout<<"PD"<< PD<< std::endl;
		comm_pub_.p_1.push_back(Pid);
		comm_pub_.p_2.push_back(-Pid);

		pub_comm_.publish(comm_pub_);

		vel_pub.x = vel1;
		vel_pub.y = vel2;

		pub_vel_.publish(vel_pub);
		std::cout<<"P:"<< P<<" "<<"I:"<< I<< " "<<"D:"<< D<< std::endl;
		// std::cout<<"enc1_:"<< enc1_g<<"   "<<"enc2_:"<< enc2_g<< std::endl;

		// to_matlab.z = PD;
		// pub_to_matlab.publish(to_matlab);



	}
	





}

int PID::sgn(double d){
    return d<0? -1 : d>0; 
    }

double PID::unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,previousAngle);
}

double PID::angleDiff(double a,double b){
    double dif = std::fmod(b - a + PI,PI2);
    if (dif < 0)
        dif += PI2;
    return dif - PI;
}




