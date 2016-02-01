#ifndef ARROW_H
#define ARROW_H

#include <string.h>

class Arrow {
	
	public:
		Arrow();
		Arrow(double d, int p, double t);
		double getDegree();
		int getPosition();	
		double getTimestamp();
		void setDegree(double d);
		void setPosition(int p);
		void setTimestamp(double t);
	private:
		double degree;
		int position;
		double timestamp;
	
};
#endif
