#include <Sensor.h>

Sensor::Sensor()
{
	//Topic you want to publish
    pub_gyro_ = n_.advertise<geometry_msgs::Vector3>("gyro_good", 10);
    pub_acc_ = n_.advertise<geometry_msgs::Vector3>("acc_good", 10);
    pub_q_est_= n_.advertise<geometry_msgs::Quaternion>("q_est", 10);
    pub_euler_ = n_.advertise<geometry_msgs::Vector3>("RPY", 10);
    // pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_class/cube_ref", 10);

	//Topic you want to subscribe
    sub_imu_acc_ = n_.subscribe("/qb_class_imu/acc", 10, &Sensor::callback_imu_acc, this);
    sub_imu_gyro_ = n_.subscribe("/qb_class_imu/gyro", 10, &Sensor::callback_imu_gyro, this);
    step_ = 0;
    n_sample_ = 100;

    data_1_ = Eigen::MatrixXd::Zero(n_sample_,3);
    data_2_ = Eigen::MatrixXd::Zero(n_sample_,3);

    flag_run1_ = flag_run2_ =  false;
    flag_offset_ = true;

    q_old_.w() = 1.0; 
    q_old_.vec() << 0, 0.0, 0.0;  
	

}

Sensor::~Sensor()
{

}


//tutte le acc provenienti dalle imu (NON MYO)
void Sensor::callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
  	acc_1_ << msg->m[0].x, msg->m[0].y, msg->m[0].z;
  	acc_2_ << msg->m[1].x, msg->m[1].y, msg->m[1].z;
  	flag_run1_ = true;

}

void Sensor::callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
  	gyro_1_ << msg->m[0].x, msg->m[0].y, msg->m[0].z;
  	gyro_2_ << msg->m[1].x, msg->m[1].y, msg->m[1].z;
  	flag_run2_ = true;

}

void Sensor::offset_gyro(Eigen::Vector3d gyro1, Eigen::Vector3d gyro2)
{
	int j;
	Eigen::Vector3d diff_1, diff_2;

	Eigen::Quaterniond gyro_q;
	
	

	if(flag_run1_ && flag_run2_)
	{
		if(step_ == 0)
		{
			data_1_(step_,0) = gyro1(0);
			data_1_(step_,1) = gyro1(1);
			data_1_(step_,2) = gyro1(2);
			data_2_(step_,0) = gyro2(0);
			data_2_(step_,1) = gyro2(1);
			data_2_(step_,2) = gyro2(2);
		// std::cout<< "sn qui:"<<std::endl;
		// std::cout<< "off1:"<< data_1(step_,0)<<" "<<data_1(step_,1)<<" "<<data_1(step_,2)<<std::endl;
			step_++;

		}
		else
		{	
			if(step_ < 100)
			{
				diff_1(0) =  (gyro1(0) - data_1_(0,0));
				diff_1(1) =  (gyro1(1) - data_1_(0,1));
				diff_1(2) =  (gyro1(2) - data_1_(0,2));
				diff_2(0) =  (gyro2(0) - data_2_(0,0));
				diff_2(1) =  (gyro2(1) - data_2_(0,1));
				diff_2(2) =  (gyro2(2) - data_2_(0,2));

				diff_1 = diff_1.cwiseAbs();
				diff_2 = diff_2.cwiseAbs();

				// diff_1(0) =  diff_1(0).norm();
				// diff_1(1) =  diff_1(1).norm();
				// diff_1(2) =  diff_1(2).norm();
				// diff_2(0) =  diff_2(0).norm();
				// diff_2(1) =  diff_2(1).norm();
				// diff_2(2) =  diff_2(2).norm();


				for(j = 0; j < 3; j++)
				{
					if(diff_1(j) < 2)
					{
						data_1_(step_, j) = gyro1(j);
						gyro1_old_(j) = gyro1(j);
					}
					else
					{
						data_1_(step_, j) = gyro1_old_(j);
					}
				}

				for(j = 0; j < 3; j++)
				{
					if(diff_2(j) < 2)
					{
						data_2_(step_, j) = gyro2(j);
						gyro2_old_(j) = gyro2(j);
					}
					else
					{
						data_2_(step_, j) = gyro2_old_(j);
					}
				}
				step_++;
				// std::cout<< "sn qui:"<<step_<<std::endl;
			}
			else 
			{	
				offset_1_(0) = data_1_.col(0).sum() / n_sample_;
				offset_1_(1) = data_1_.col(1).sum() / n_sample_;
				offset_1_(2) = data_1_.col(2).sum() / n_sample_;
				offset_2_(0) = data_2_.col(0).sum() / n_sample_;
				offset_2_(1) = data_2_.col(1).sum() / n_sample_;
				offset_2_(2) = data_2_.col(2).sum() / n_sample_;

				flag_offset_ = false;
			 std::cout<< "off1:"<< offset_1_(0)<<" "<<offset_1_(1)<<" "<<offset_1_(2)<<std::endl;
			 std::cout<< "off2:"<< offset_2_(0)<<" "<<offset_2_(1)<<" "<<offset_2_(2)<<std::endl;

			}
			

		}


	}
}

void Sensor::run()
{
	double Beta = 0.03; //035 più basso è più mi fido del gyro
	int i;
	double tr_g = 1.5;
	double tr_a = 0.1;

	geometry_msgs::Vector3 gyro_g_pub, acc_g_pub, euler_pub;
	Eigen::Vector3d diff_g, diff_a;

	//sono i gyro senza offset
	Eigen::Vector3d gyro_1_0, gyro_2_0;

	Eigen::Vector3d acc_g, gyro_g;
	Eigen::Vector3d Acc_earth(0.0, 0.0, 1.0);
	Eigen::VectorXd F_partial(3);
	Eigen::MatrixXd J_partial(3, 4);
	Eigen::VectorXd step_q(4);
	Eigen::Quaterniond q_step;
	Eigen::Quaterniond q_ang_rate;
	Eigen::Quaterniond q_nabla;
	Eigen::Quaterniond q_integr;
	Eigen::Quaterniond q_est;
	Eigen::Quaterniond qDot;
	Eigen::Quaterniond gyro_q;
	Eigen::Vector3d euler;


	if(flag_offset_)
	{
		offset_gyro(gyro_1_, gyro_2_);
	}
	else
	{
		gyro_1_0 = gyro_1_ - offset_1_;
		gyro_2_0 = gyro_2_ - offset_2_;

		diff_g = (gyro_1_0 - gyro_2_0);
		diff_g = diff_g.cwiseAbs();

		for(i = 0; i < 3; i++)
		{
			if(diff_g(i) < tr_g)
			{
				gyro_g(i) = (gyro_1_0(i) + gyro_2_0(i)) / 2;
				gyro_old_(i) =  gyro_g(i);
			}
			else
			{
				gyro_g(i) = gyro_old_(i);
			}
		}

		// gyro_1_0 = gyro_1_ - offset_1_;
		// gyro_2_0 = gyro_2_ - offset_2_;

		diff_a = (acc_1_ - acc_2_);
		diff_a = diff_a.cwiseAbs();

		for(i = 0; i < 3; i++)
		{
			if(diff_a(i) < tr_a)
			{
				acc_g(i) = acc_1_(i);
				// acc_g(i) = (acc_1_(i) + acc_2_(i)) / 2;
				acc_old_(i) =  acc_g(i);
			}
			else
			{
				acc_g(i) = acc_old_(i);
			}
		}


		//Normalizzazione acc
		acc_g = acc_g / acc_g.norm();

		// for(i = 0; i < 3; i++)
		// {
		// 	if(abs(gyro_g(i)) < 0.001)
		// 	{
		// 		gyro_g(i) = 0;	
		// 	}
		// }



		//gradi to rad al sec
		gyro_g = gyro_g * (PI / 180);
		gyro_q.w() = 0;
		gyro_q.vec() = gyro_g;

		// std::cout<<"gyro"<<gyro_g(0)<<" "<<gyro_g(1)<<" "<< gyro_g(2)<<std::endl;
		// getchar();

		///////////////////////////////
		double q1 = q_old_.w();
		double q2 = q_old_.x();
		double q3 = q_old_.y();
		double q4 = q_old_.z();

		double d1x = Acc_earth(0);
		double d1y = Acc_earth(1);
		double d1z = Acc_earth(2);

		double s1x = acc_g(0);
		double s1y = acc_g(1);
		double s1z = acc_g(2);

		F_partial <<	2*d1x*(0.5 - q3*q3 - q4*q4) + 2*d1y*(q1*q4 + q2*q3) + 2*d1z*(q2*q4 - q1*q3) - s1x, 
				2*d1x*(q2*q3 - q1*q4) + 2*d1y*(0.5 - q2*q2 - q4*q4) + 2*d1z*(q1*q2 + q3*q4) - s1y,
				2*d1x*(q1*q3 - q2*q4) + 2*d1y*(q3*q4 - q1*q2) + 2*d1z*(0.5 - q2*q2 - q3*q3) - s1z;
		

		J_partial <<  2*d1y*q4-2*d1z*q3  ,   2*d1y*q3+2*d1z*q4             ,     -4*d1x*q3+2*d1y*q2-2*d1z*q1      ,      -4*d1x*q4+2*d1y*q1+2*d1z*q2,  
	    	         -2*d1x*q4+2*d1z*q2  ,   2*d1x*q3-4*d1y*q2+2*d1z*q1    ,      2*d1x*q2+2*d1z*q4               ,      -2*d1x*q1-4*d1y*q4+2*d1z*q3,
	      	          2*d1x*q3-2*d1y*q2  ,   2*d1x*q4-2*d1y*q1-4*d1z*q2    ,      2*d1x*q1+2*d1y*q4-4*d1z*q3      ,                2*d1x*q2+2*d1y*q3;
	      
	    step_q = J_partial.transpose() * F_partial;

	    step_q = step_q / step_q.norm();

	    q_step.w() = step_q(0);
		q_step.x() = step_q(1);
		q_step.y() = step_q(2);
		q_step.z() = step_q(3);

		q_ang_rate = (q_old_ * gyro_q);
		q_ang_rate.w() = 0.5 * q_ang_rate.w();
		q_ang_rate.vec() = 0.5 * q_ang_rate.vec();

		q_nabla.w() = Beta * q_step.w();
		q_nabla.vec() = Beta * q_step.vec();

		//calcolo di qDot
		qDot.w() = q_ang_rate.w() - q_nabla.w();
		qDot.vec() = q_ang_rate.vec() - q_nabla.vec();

		///////////////////////////////////////
		q_integr.w() = qDot.w() * dt;
		q_integr.vec() = qDot.vec() * dt;

		q_est.w() = q_old_.w() + q_integr.w();
		q_est.vec() = q_old_.vec() + q_integr.vec();

		q_est.normalize();

		q_old_ = q_est;

		// Eigen::Vector3d euler = q_est.toRotationMatrix().eulerAngles(0, 2, 1);
		euler (1) = asin(2 * (q_est.w() * q_est.y()) - (q_est.x() * q_est.z()));
  		euler (0) = 0.0;
  		euler (2) = 0.0;










		gyro_g_pub.x = gyro_g(0);
		gyro_g_pub.y = gyro_g(1);
		gyro_g_pub.z = gyro_g(2);

		pub_gyro_.publish(gyro_g_pub);

		acc_g_pub.x = acc_g(0);
		acc_g_pub.y = acc_g(1);
		acc_g_pub.z = acc_g(2);

		pub_acc_.publish(acc_g_pub);

		q_est_pub_.w = q_est.w();
		q_est_pub_.x = q_est.x();
		q_est_pub_.y = q_est.y();
		q_est_pub_.z = q_est.z();

		pub_q_est_.publish(q_est_pub_);

		euler_pub.x = euler(0);
		euler_pub.y = euler(1);
		euler_pub.z = euler(2);

		pub_euler_.publish(euler_pub);



	}
}

