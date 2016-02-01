#ifndef IMPORT_H
#define IMPORT_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
//#include <sound.h>
#include "Arrow.h"

bool checkFileExists(const std::string& name);
void split(const std::string& s, char c, std::vector<std::string>& v);
std::vector<Arrow> readFile(std::string dfilename);
#endif
