
/*
These functions are used in RFSniffer to receive and validate
incoming messages. In the first section the support functions themselves
are found, below are the corresponding tests.

https://www.learncpp.com/cpp-tutorial/header-files/
*/

std::string given_sum_string;
std::string str_at_index;
std::string get_msg;
int given_sum;
int received_sum;
int nr_at_index;
int mod_received_sum;
int converted_nr;

using namespace std;  // don't know what this is  


//============================================
//			SUPPORT FUNCTIONS 

int get_int_from_str(string string_raw, int index) {
	spdlog::debug("Running get_int_from_str()");
	
	get_msg = string_raw.at(index);
	std::istringstream iss(get_msg);	// Get the string of the single number at the index
	iss >> converted_nr;							// Get the number in integer form

	spdlog::debug("Function get_int_from_str() returns: {}", converted_nr); 
	return converted_nr;			
}


bool check_msg(string msg_str) {
	// This function checks if the message has the right checksum. It does
	// so by checking if the the last four numbers sum to the amount specified 
	// in the second and third number. For example, message 1150203 is seen as
	// 1 1 5 0203. It is the first message (1) of rover 1 (1) with checksum 5, which checks out
	// as the sum of 0 2 0 3 = 5. A modulus of 10 is applied to keep the number as a 
	// single digit.

	spdlog::debug("Running check_msg()");
	spdlog::info("Received msg_str: {}", msg_str.c_str());

	// Make sure variables are clear	
	given_sum_string = "";
	received_sum = 0;
	given_sum = 0;

	// 			RETRIEVING RECEIVED CHECKSUM
	// Get the checksum from the message, which is the number at index 2
	given_sum = get_int_from_str(msg_str, 2);
	spdlog::info("Given checksum: {}", given_sum);

	//			CALCULATING CHECKSUM FROM MESSAGE
	// Use for loop to get the checksum of the received message (https://www.tutorialspoint.com/cplusplus/cpp_for_loop.htm)
	// This for loop should run from 3 to 6
	for (int index = 3; index < 7; index += 1) {
		
		nr_at_index = get_int_from_str(msg_str, index);	// Get the number in integer form
		received_sum += nr_at_index;		// Add the number to the checksum
		spdlog::debug("Received sum: {}", received_sum);
	}
	
	mod_received_sum = received_sum % 10;
	spdlog::info("Calculated checksum: {}", mod_received_sum);

	
	//			CHECKSUM COMPARISON

	// Checksums equal
	if (mod_received_sum == given_sum) {
		spdlog::info("Checksums match, message accepted");
		return true;
	}
	
	// Checksums not equal
	else if (mod_received_sum != given_sum) {
		spdlog::info("Checksums don't match, message ignored");
		return false;
	}
	
	// Any other situation is also considered faulty and will be ignored
	else {
		spdlog::info("Something went wrong in checksum comparison, message ignored");
		return false;
	}
};



//============================================
//					TESTS

bool test_get_int_from_str() {
	spdlog::info("Running test_check_msg()");
	std::string test_string = "1256450";
	if (get_int_from_str(test_string, 0) == 1 && get_int_from_str(test_string, 1) == 2 && get_int_from_str(test_string, 2) == 5 && get_int_from_str(test_string, 3) == 6 && get_int_from_str(test_string, 4) == 4 && get_int_from_str(test_string, 5) == 5 && get_int_from_str(test_string, 6) == 0) {
		return true;
	}
	else {
		return false;
	}
}

bool test_check_msg() {
	spdlog::info("Running test_check_msg()");
	std::string check_str;

	check_str = "1256450";	// This message is valid
	bool check1 = check_msg(check_str);// Should be true
	
	check_str = "1256454";	// This message is invalid
	bool check2 = check_msg(check_str);// Should be false

	check_str = "9300000";
	bool check3 = check_msg(check_str);// Should be true

	check_str = "1700091";
	bool check4 = check_msg(check_str);// Should be true

	check_str = "6590090";
	bool check5 = check_msg(check_str);// Should be true
	
	check_str = "3891099";
	bool check6 = check_msg(check_str);// Should be true

	check_str = "3891699";
	bool check7 = check_msg(check_str);// Should be false

	if (check1 == true && check2 == false && check3 == true && check4 == true && check5 == true && check6 == true && check7 == false) {
		return true;
	}
	else {
		return false;
	}
};