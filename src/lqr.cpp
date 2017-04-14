#include <lqr.h>

lqr::lqr()
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
    sub_gyro_= n_.subscribe("/gyro_good", 10, &lqr::callback_gyro, this);
    sub_euler_ = n_.subscribe("/RPY", 10, &lqr::callback_imu_euler, this);
    // sub_gain_ = n_.subscribe("/gain_lqr", 10, &lqr::callback_gain_lqr, this);
    // sub_corr = n_.subscribe("/qb_class/cube_current", 10, &lqr::callback_corr, this);
    sub_enc = n_.subscribe("/qb_class/cube_measurement", 10, &lqr::callback_meas, this);
    // sub_gain_v_ = n_.subscribe("/gain_lqr_v", 10, &lqr::callback_gain_lqr_v, this);
    // sub_gain_w_ = n_.subscribe("/gain_lqr_w", 10, &lqr::callback_gain_lqr_w, this);
    // sub_comm = n_.subscribe("/comm", 10, &lqr::callback_comm, this);
    // sub_cube_ = n_.subscribe("/ref_cube", 10, &lqr::callback_cube, this);
    // sub_myo_ = n_.subscribe("/For_r_imu", 10, &lqr::callback_myo, this);



    com_R = 0;
    com_L = 0;



    flag_run1_ = flag_run2_ = flag_run3_ =  false;
    usleep(3000000);
    
    // kp_ = kd_ = ki_ = 0.0;
    // kp_v_ = kd_v_ = ki_v_ = 0.0;
    // kp_w_ = kd_w_ = ki_w_ = 0.0;
    // th_eq_ = 0.0;
    // th_pr_ = 0.6 * (65536 /(4 * PI));

    // gyro_ = Eigen::Vector3d::Zero();
    // euler_ = Eigen::Vector3d::Zero();
    

	

}

lqr::~lqr()
{

}


//tutte le acc provenienti dalle imu (NON MYO)
void lqr::callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	gyro_ << msg->x, msg->y, msg->z;
  	flag_run1_ = true;

}

void lqr::callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	euler_ << msg->x, msg->y, msg->z;
  	flag_run2_ = true;

}

// void lqr::callback_gain_lqr(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//   	kp_ = msg->x;
//   	ki_ = msg->y;  	
//   	kd_ = msg->z;
// }

// void lqr::callback_corr(const qb_interface::cubeCurrent::ConstPtr& msg)
// {
// 	to_matlab.x = msg->current_m1[0];
// }

// void lqr::callback_gain_lqr_v(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//   	kp_v_ = msg->x;
//   	ki_v_ = msg->y;  	
//   	kd_v_ = msg->z;
// }

void lqr::callback_meas(const qb_interface::cubePos::ConstPtr& msg)
{
	// to_matlab.y = msg->p_1[0];
	if(!flag_run3_)
	{
		enc1_ = -msg->p_1[0] * 4.0 * PI / 65536;
		enc2_ = -msg->p_2[0] * 4.0 * PI / 65536;
		enc1_of_ = enc1_;
		enc2_of_ = enc2_;
		// enc1_old_ = enc1_;
		// enc2_old_ = enc2_;
		enc1_ = 0;
		enc2_ = 0;
		
		flag_run3_ = true;
	}
	enc1_ = -msg->p_1[0] * 4.0 * PI / 65536  - enc1_of_;
	enc2_ = -msg->p_2[0] * 4.0 * PI / 65536  - enc2_of_;


	
	// encL_cube_ = msg->p_L[1] * 4.0 * PI / 65536;
	
	
	
}

// void lqr::callback_comm(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//   	vel_rif_ = msg->x;
//   	w_rif_ = msg->y;  	
  	
// }

// void lqr::callback_gain_lqr_w(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//   	kp_w_ = msg->x;
//   	ki_w_ = msg->y;  	
//   	kd_w_ = msg->z;
// }

// void lqr::callback_cube(const geometry_msgs::Vector3::ConstPtr& msg)
// {
// 	// chi pubblica deve pubblicare in rad
//   	th_eq_ = msg->x * (65536 /(4 * PI));
//   	th_pr_ = msg->y * (65536 /(4 * PI));  	
//   	// kd_v_ = msg->z;
// }

// void lqr::callback_myo(const sensor_msgs::Imu::ConstPtr& msg)
// {
// 	q_myo_.w = msg->orientation.w;
// 	q_myo_.x = msg->orientation.x;
// 	q_myo_.y = msg->orientation.y;
// 	q_myo_.z = msg->orientation.z;

// 	th_eq_ = asin(2 * (q_myo_.w * q_myo_.y - (q_myo_.x * q_myo_.z))) * (65536 /(4 * PI));
// }


void lqr::run()
{
	double th = 0.0;
	double raggio = 0.12;
	double lunghezza = 0.28;
	double offset_pitch =  0.0;//0.0375;
	qb_interface::cubeRef comm_pub_;
	double enc1_g, enc2_g, vel1_enc, vel2_enc, vel1, vel2, vel, pos1, pos2, pos, a, w, yaw;
	geometry_msgs::Vector3 vel_pub;

	double lqr_v_der, vel_der, w_der;
	
	Eigen::MatrixXd k(2,6);
	Eigen::MatrixXd command(2,1);
	Eigen::MatrixXd state(6,1);

	// k << 0.0, -10.0, -3720.5, -4517.6,
	// 	-10.0, 0.0, -3720.5, -4517.6;
	k << 0.0, -10.0,-1.5, -15.9, -717.7, -90.8,
		-10.0, 0.0,-1.5, -15.9, -717.7, -90.8;
		// k << 0.0, -34.6,-5.17, -19.32, -886.0, -110.0,
		// -34.6, 0.0,-5.17, -19.32, -886.0, -110.0;
		// k << 0.0, 0.0, 0.0, 0.0,
		// -0.0, 0.0, 0.0, 0.0;
	// offset_pitch = atan2(((0.09*0.380*sin(encL_cube_))/(0.790+0.380+0.380)),(((0.1*0.380)+((0.09*cos(encL_cube_)+0.1)*0.380))/(0.790+0.380+0.380)));

	a = dt / (0.1 + dt);

	

	if(flag_run1_ && flag_run2_ && flag_run3_)
	{

		// enc1_ = enc1_ * 4.0 * PI / 65536;
		// enc2_ = enc2_ * 4.0 * PI / 65536;


		//posizione e velocità relativa struttura ruote
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

		// posizine/velocità assoluta ruote
		pos1 = euler_(1) + enc1_g;
		pos2 = euler_(1) - enc2_g;

		vel1 = gyro_(1) + vel1_enc;
		vel2 = gyro_(1) - vel2_enc;

		// vel1 = vel1_enc;
		// vel2 = vel2_enc;


		vel1 = (1 - a) * vel1_old_ + (a * vel1);
		vel2 = (1 - a) * vel2_old_ + (a * vel2);

		vel1_old_ = vel1;
		vel2_old_ = vel2;

		//velocità lineare/ posizione
		vel = (vel1 + vel2) * raggio / 2;
		pos = (pos1 + pos2) * raggio / 2;

		//velocità angolare / imbardata
		w = (vel1 - vel2) * raggio / lunghezza;
		yaw = (pos1 - pos2) * raggio / lunghezza;

		//potresti prendere l'accelerazione dell'accelerometro
		vel_der = (vel - vel_old_)/dt;
		vel_old_ = vel;

		w_der = (w - w_old_)/dt;
		w_old_ = w; 


		//controllo LQR

		// pos_rif_ += vel_rif_ * dt;
		// //controllo doppio loop
		// P_v = kp_v_ * (vel_rif_ - vel);
		// I_v = ki_v_ * (pos_rif_ - pos);
		// // D_v = kd_v_ * (-vel_der);
		// D_v += kd_v_ * (pos_rif_ - pos)/ dt;

		// lqr_v = P_v + I_v + D_v;

		// lqr_v_der = (lqr_v - lqr_v_old)/dt;
		// lqr_v_old = lqr_v;

		// P = kp_ * (lqr_v - euler_(1) - offset_pitch);
		// if((P * sgn(P)) <= th)
		// {
		// 	P = th * sgn(P);
		// }

		// I += ki_ *(lqr_v - euler_(1) - offset_pitch) / 200;


		// D = kd_ * (lqr_v_der-gyro_(1));

		// Pid = P + I + D;


		// yaw_rif_ += w_rif_ * dt;
		// //controllo imbarbdata
		// P_w = kp_w_ * (w_rif_ - w);
		// I_w = ki_w_ * (yaw_rif_ - yaw);
		// D_w = kd_w_ * (-w_der);

		// lqr_w = P_w + I_w + D_w;




		
		// //solo loop interno
		// P = kp_ * (-euler_(1));
		// if((P * sgn(P)) <= th)
		// {
		// 	P = th * sgn(P);
		// }

		// I += ki_ *(-euler_(1)) / 200;
		// D = kd_ * (-gyro_(1));

		// Pid = P + I + D;

		// non combacia con  l'articolo perchè i motori hanno direzioni differenti
		// com_R = Pid + lqr_w;
		// com_L = -Pid + lqr_w;
		state << pos1, pos2, vel1, vel2, euler_(1), gyro_(1);
		command = k * state;
		com_R = command(0,0);
		com_L = command(1,0);
		// com_R = com_R *100 /12;
		// com_L = com_L *100 /12;

		
		

		if(com_R < -100)
		{
			com_R = -100;
		}
		
		if (com_R > 100)
		{
			com_R = 100;
		}

		if(com_L < -100)
		{
			com_L = -100;
		}
		
		if (com_L > 100)
		{
			com_L = 100;
		}
		// std::cout << "PD4:" << PD << std::endl;

		// //calcolo th_eq e preset per il cubo
		// m1_cube_ = th_eq_ + th_pr_;
		// m2_cube_ = th_eq_ - th_pr_;

		 

		

		// std::cout<<"PD"<< PD<< std::endl;

		//comando ruote
		comm_pub_.p_1.push_back(-com_R);
		comm_pub_.p_2.push_back(com_L);
		// //comando cubo
		// comm_pub_.p_1.push_back(m1_cube_);
		// comm_pub_.p_2.push_back(m2_cube_);


		pub_comm_.publish(comm_pub_);

		vel_pub.x = vel1;
		vel_pub.y = vel2;

		pub_vel_.publish(vel_pub);
		// std::cout<<"P:"<< P<<" "<<"I:"<< I<< " "<<"D:"<< D<< std::endl;
		// std::cout<<"enc1_:"<< enc1_g<<"   "<<"enc2_:"<< enc2_g<< std::endl;

		// to_matlab.z = PD;
		// pub_to_matlab.publish(to_matlab);



	}
	





}

int lqr::sgn(double d){
    return d<0? -1 : d>0; 
    }

double lqr::unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,previousAngle);
}

double lqr::angleDiff(double a,double b){
    double dif = std::fmod(b - a + PI,PI2);
    if (dif < 0)
        dif += PI2;
    return dif - PI;
}





