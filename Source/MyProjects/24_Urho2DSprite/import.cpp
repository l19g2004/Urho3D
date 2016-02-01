 #include <import.h>

using namespace std;

bool checkFileExists(const std::string& name) {
    ifstream f(name.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }   
}
	
void split(const string& s, char c, vector<string>& v) {
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}

vector<Arrow> readFile(string dfilename) {
	ifstream datafile(dfilename);
    std::vector<Arrow> arrowList;
	
	if (datafile.is_open()) {
		string line;
        Arrow newArrow = Arrow();
		while (getline(datafile, line)) {
            //cout << line << endl;
			if (line.find("position") != std::string::npos) {
				vector<string> temp;
				split(line, ':', temp);
                newArrow.setPosition(stoi(temp.at(1)));
			}
			if (line.find("degree") != std::string::npos) {
				vector<string> temp;
				split(line, ':', temp);
                newArrow.setDegree(stod(temp.at(1)));
			}
			if (line.find("timestamp") != std::string::npos) {
				vector<string> temp;
				split(line, ':', temp);
                newArrow.setTimestamp(stod(temp.at(1)));
			}
			if (line.find("--") != std::string::npos) {
				arrowList.push_back(newArrow);
			}
		}
		datafile.close();
	}
	return arrowList;
	
}

