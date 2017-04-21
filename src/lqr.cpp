#include <lqr.h>

lqr::lqr()
{
	//Topic you want to publish
    pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_class/cube_ref", 10);
    pub_vel_ = n_.advertise<geometry_msgs::Vector3>("vel", 10);

	//Topic you want to subscribe
    sub_gyro_= n_.subscribe("/gyro_good", 10, &lqr::callback_gyro, this);
    sub_euler_ = n_.subscribe("/RPY", 10, &lqr::callback_imu_euler, this);
    sub_enc = n_.subscribe("/qb_class/cube_measurement", 10, &lqr::callback_meas, this);
    sub_myo_ = n_.subscribe("/For_r_imu", 10, &lqr::callback_myo, this);




    com_R = 0;
    com_L = 0;
    th_eq_ = 0.0;
    th_pr_ = 0.6 * (65536 /(4 * PI));



    flag_run1_ = flag_run2_ = flag_run3_ =  false;
    usleep(3000000); 

	

}

lqr::~lqr()
{

}



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

void lqr::callback_myo(const sensor_msgs::Imu::ConstPtr& msg)
{
	q_myo_.w = msg->orientation.w;
	q_myo_.x = msg->orientation.x;
	q_myo_.y = msg->orientation.y;
	q_myo_.z = msg->orientation.z;

	th_eq_ = asin(2 * (q_myo_.w * q_myo_.y - (q_myo_.x * q_myo_.z))) * (65536 /(4 * PI));
}




void lqr::run()
{
	double th = 35.0;
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
	k << 0.0, -10.0,-1.5, -15.6, -735.0, -90.8,
		-10.0, 0.0,-15.6, -1.5, -735.0, -90.8;
		// k << 0.0, -3.0,-0.5, -3.6, -179.0, -27.0,
		//     -3.0, 0.0,-3.6, -0.5, -179.0, -27.0;
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

		
		state << pos1, pos2, vel1, vel2, euler_(1), gyro_(1);
		command = k * state;

		// if((euler_(1) * k(0,4) * sgn(euler_(1) * k(0,4))) <= th)
		// {
		// 	command(0,0) = command(0,0) - (euler_(1) * k(0,4)) + th * sgn(euler_(1) * k(0,4));
		// 	command(1,0) = command(1,0) - (euler_(1) * k(1,4)) + th * sgn(euler_(1) * k(1,4));
		// }

		


		// if((command(0,0) * sgn(command(0,0))) <= th)
		// {
		// 	command(0,0) = th * sgn(command(0,0));
		// }

		// if((command(1,0) * sgn(command(1,0))) <= th)
		// {
		// 	command(1,0) = th * sgn(command(1,0));
		// }

		com_R = command(0,0);
		com_L = command(1,0);

		
		

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

		//calcolo th_eq e preset per il cubo
		m1_cube_ = th_eq_ + th_pr_;
		m2_cube_ = th_eq_ - th_pr_;
		

		//comando ruote
		comm_pub_.p_1.push_back(-com_R);
		comm_pub_.p_2.push_back(com_L);

		//comando cubo
		comm_pub_.p_1.push_back(m1_cube_);
		comm_pub_.p_2.push_back(m2_cube_);



		pub_comm_.publish(comm_pub_);

		// vel_pub.x = vel1;
		// vel_pub.y = vel2;

		// pub_vel_.publish(vel_pub);
		



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





