#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <vector>

#include <sys/stat.h>

void usage() {
	printf("Usage: ./xmlgen -N Name -x XML -n Num -s Seed -t Test -o outfile -S seeds -T Expr-Label\n");
  std::cout << "Where:\n"
               "\t-x - Master Xml\n"
               "\t-n - Number of each experiment to run\n"
               "\t-t - Name of the experiment labels and output directory\n"
               "\t-T - Name of a single experiment to be run\n"
               "\t-o - Ouput xml file\n"
               "\t-s - Seed for number generator\n"
               "\t-S - Seeds to use\n"
               "\t-N - (Optional) Modifier for the out names and strategy label\n";
}

void write_solver_node(std::ofstream& _out,
	const std::string& _strategyLabel,
	const std::string& _baseFile,
	size_t _seed);

int main(int argc, char** argv) {
	
	if(argc < 11) {
		usage();
		exit(1);
	}

	std::string masterXml;
	std::string masterBuffer;
	std::string outFilename;
	std::string expFile;
	std::string name = "";
  std::string seedsFile = "";

  bool useSeedsFile = false;
  bool singleExpr = false;

	int seed;
	int numTests;

	
	std::vector<size_t> seeds;
	std::vector<std::string> experiments;


	int c;
	opterr = 0;

	while((c = getopt(argc, argv, "x:n:s:t:o:N:S:T:")) != -1) {
		switch(c) {
			case 'x':
				masterXml = std::string(optarg);
				break;
			case 'n':
				numTests = atoi(optarg);
				break;
			case 's':
				seed = atoi(optarg);
				break;
			case 't':
				expFile = std::string(optarg);
				break;
			case 'o':
				outFilename = std::string(optarg);
				break;
			case 'S':
				seedsFile = std::string(optarg);
        useSeedsFile = true;
				break;
			case 'N':
				name = std::string(optarg);
				break;
      case 'T':
        experiments.push_back(std::string(optarg));
        singleExpr = true;
        break;
			default:
				usage();
				exit(1);
		}
	}


	{ // generating numTests random seeds
    if(useSeedsFile) {
		  std::ifstream fis(seedsFile.c_str());
      if(!fis){
        // todo: error handling
        std::cerr << "Error: unable to open file '" << expFile << "'" << std::endl;
        exit(1);
      }
      size_t s;
      while(fis >> s) seeds.push_back(s); 

      if(seeds.size() > (size_t) numTests)
        while(seeds.size() > (size_t) numTests) seeds.pop_back();
      else if(seeds.size() < (size_t) numTests) {
        std::cerr << "Error: not enough seeds given in file: requested: " << numTests << " in file: " << seeds.size() << std::endl;
        exit(1);
      }
    }
    else {
      std::default_random_engine gen(seed);
      std::uniform_int_distribution<size_t> distribution(1000000, 9999999);
      std::ofstream seed("seeds.txt");
      int temp = numTests;
      while(temp--) {
        seeds.push_back(distribution(gen));
        seed << seeds.back() << std::endl;
      }  
    }
  }


	{ // reads experiment file
    if(!singleExpr) {
      std::ifstream fis(expFile.c_str());

      if(!fis){
        // todo: error handling
        std::cerr << "Error: unable to open file '" << expFile << "'" << std::endl;
        exit(1);
      }

      std::string temp;
      while(fis >> temp) {
        experiments.push_back(temp);
        
        mkdir(temp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      } 
    }      
  }

  std::cout << "Generating XML: " << std::endl
            << "Experiments:\n";
  for(const auto& e : experiments)
    std::cout << "\t" << e << std::endl; 
  std::cout << "Num: " << numTests << std::endl
            << "Master XML: " << masterXml << std::endl
            << "Seed: " << seed << std::endl
            << "Output File: " << outFilename << std::endl;

	{ // reads master xml
		std::ifstream ifs(masterXml.c_str());

		if(!ifs) {
			std::cerr << "Error: unable to open file '" << masterXml << "'" << std::endl;
			exit(1);
		}

		std::stringstream ss;
		ss << ifs.rdbuf();

		masterBuffer = ss.str();
	}

	// generating the new xml
	const std::string refString = "</MPStrategies>";
	size_t index = masterBuffer.find(refString);
  if(index == std::string::npos) {
    std::cerr << "Error: failed to find '</MPStrategies>' tag\n";
    exit(1);
  }

	index += refString.size() + 1;
	std::ofstream ofs(outFilename.c_str());
	std::string footer = masterBuffer.substr(index + 1);
	if(!ofs) {
			std::cerr << "Error: unable to open file '" << outFilename << "'" << std::endl;
			exit(1);
	}

	ofs.clear(); // clear the current content of the file.

	ofs << masterBuffer.substr(0, index) << std::endl;
	for(const auto& exp : experiments) {
		for(int i = 0; i < numTests; ++i) {
			std::string baseFilename = exp + '/' + exp + (!name.empty() ? "." + name : "");
      std::string ex = exp + (!name.empty() ? name : "");
			write_solver_node(ofs, ex, baseFilename, seeds[i]);
		}
	}

	ofs << footer;

	// this is not the best way to achieve this.
	// Maybe it would be better to link with tinyxml
	// and use that to write the xml.
	// I will investigate.
	return 0;
}

void write_solver_node(std::ofstream& _out,
	const std::string& _strategyLabel,
	const std::string& _baseFile,
	size_t _seed) {
	char buffer[1024];
	const char* fmt = "<Solver mpStrategyLabel=\"%s\" seed=\"%lu\" baseFilename=\"%s\" vizmoDebug=\"false\"/>";
	sprintf(buffer, fmt, _strategyLabel.c_str(), _seed, _baseFile.c_str());
	_out << "\t" << buffer << std::endl;
}
