float get_A_angle(float x, float y)
{
	float theta2 = acos((x**2 + y**2 - 3.75**2 - 2.5**2) / (2*3.75*2.5));
	float theta4 = -theta2;
	float theta1 = atan2(y,x) - asin(2.5*sin(theta2)/sqrt(x**2 + y**2));
	float theta3 = atan2(y,x) - asin(2.5*sin(theta4)/sqrt(x**2 + y**2));
	if (theta1 > pi || theta1 < 0) {
  	return theta3;
  }
  else {
  	return theta1;
  }
}

float get_B_angle(float x, float y)
{
	float theta2 = acos((x**2 + y**2 - 3.75**2 - 2.5**2) / (2*3.75*2.5));
	float theta4 = -theta2;
	float theta1 = atan2(y,x) - asin(2.5*sin(theta2)/sqrt(x**2 + y**2));
	float theta3 = atan2(y,x) - asin(2.5*sin(theta4)/sqrt(x**2 + y**2));
	if (theta1 > pi || theta1 < 0) {
  	return theta4;
  }
  else {
  	return theta2;
  }
}
