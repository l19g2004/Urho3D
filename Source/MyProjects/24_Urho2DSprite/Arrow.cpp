 #include "Arrow.h"

Arrow::Arrow() {}

Arrow::Arrow(double d, int p, double t) {
	degree = d;
	position = p;
	timestamp = t;
}

double Arrow::getDegree() {
	return degree;
}

void Arrow::setDegree(double d) {
	degree = d;	
}

int Arrow::getPosition() {
	return position;
}

void Arrow::setPosition(int p) {
	position = p;
}

double Arrow::getTimestamp() {
	return timestamp;	
}

void Arrow::setTimestamp(double t) {
	timestamp = t;	
}
