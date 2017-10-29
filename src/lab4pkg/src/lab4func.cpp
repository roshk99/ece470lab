#include "lab4pkg/lab4.h"

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
	double yaw_WgripRad, b, c, beta1, beta2, beta3, r1, r2, r3, r4;
	double b2, phi, psi; 
	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	d6 = (0.082+0.056);

	b = 0.0535;
	yaw_WgripRad = yaw_WgripDegree*PI/180.0;

	xgrip = xWgrip + 0.14;
	ygrip = yWgrip - 0.14;
	zgrip = zWgrip + 0.004;
	cout<<"xgrip: "<< xgrip<<" ygrip: "<< ygrip<<", zcen: "<<zgrip<<endl;
	xcen = xgrip + b*sin(yaw_WgripRad-PI/2);
	ycen = ygrip - b*cos(yaw_WgripRad-PI/2);
	zcen = zgrip;
	cout<<"xcen: "<< xcen<<" ycen: "<< ycen<<", zcen: "<<zcen<<endl;
	c = 0.11;
	r1 = sqrt(pow(xcen, 2) + pow(ycen, 2));
	beta1 = asin(c/r1);
	theta1 = atan2(ycen, xcen) - beta1;
	cout<<"r1: "<<r1<<"beta1: "<< beta1<<", theta1: "<< theta1<<endl;
	theta6 = PI/2 + theta1 - yaw_WgripRad;
 	
 	r2 = sqrt(pow(r1, 2)-pow(c, 2))-d4;
	x3end = r2*cos(theta1);
	y3end = r2*sin(theta1);
	z3end = zcen + d6;
	cout << "x3end: " << x3end	<< " y3end: " << y3end << " z3end: " << z3end << endl; 
	r3 = sqrt(pow(x3end, 2) + pow(y3end, 2));
	r4 = sqrt(pow(r3, 2) + pow((z3end - d1), 2));
	theta3 = PI - acos((pow(r4, 2) - pow(a2, 2) - pow(a3, 2))/(-2*a2*a3));

	beta2 = atan2((z3end - d1), r3);
	theta2 = -acos((pow(a3, 2) - pow(a2, 2) - pow(r4, 2))/ (-2*a2*r4)) - beta2;
	theta4 = -PI/2 - theta2 - theta3;
	theta5=-PI/2;
	
	// View values
	//use cout
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}
