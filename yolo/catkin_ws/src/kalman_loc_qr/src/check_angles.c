#include <stdio.h>
#include <math.h>

double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1-normalize(d1));
  else
    return(d2-normalize(d1));
}

int main()
{
	for(double a = -(2*M_PI) ; a <= (2*M_PI); a+=.1     )
		for(double b = -(2*M_PI) ; b <= (2*M_PI); b+=.1     )
		{
			printf("%f\n",angle_diff(a,b));
			if(angle_diff(a,b) != 0)
				printf("NO se pudo U_U: %f %f",a,b);
		}
	return 0;	
}
