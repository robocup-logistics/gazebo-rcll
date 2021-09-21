
/***************************************************************************
 *  machines.cpp - LLSF machine names to numbers
 *
 *  Created: Mon Mar 24 12:21:11 2014
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <utils/llsf/machines.h>

namespace llsf_utils {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

unsigned int
to_machine(std::string &machine_name, MachineAssignment machine_assignment)
{
	switch (machine_assignment) {
	case ASSIGNMENT_2013:
		if (machine_name == "M1")
			return 0;
		else if (machine_name == "M2")
			return 1;
		else if (machine_name == "M3")
			return 2;
		else if (machine_name == "M4")
			return 3;
		else if (machine_name == "M5")
			return 4;
		else if (machine_name == "M6")
			return 5;
		else if (machine_name == "M7")
			return 6;
		else if (machine_name == "M8")
			return 7;
		else if (machine_name == "M9")
			return 8;
		else if (machine_name == "M10")
			return 9;
		else if (machine_name == "D1")
			return 10;
		else if (machine_name == "D2")
			return 11;
		else if (machine_name == "D3")
			return 12;
		else if (machine_name == "TST")
			return 13;
		else if (machine_name == "R1")
			return 14;
		else if (machine_name == "R2")
			return 15;
		else
			throw fawkes::Exception("Unknown machine type %s", machine_name.c_str());

	case ASSIGNMENT_2014:
		if (machine_name == "M1")
			return 0;
		else if (machine_name == "M2")
			return 1;
		else if (machine_name == "M3")
			return 2;
		else if (machine_name == "M4")
			return 3;
		else if (machine_name == "M5")
			return 4;
		else if (machine_name == "M6")
			return 5;
		else if (machine_name == "M7")
			return 6;
		else if (machine_name == "M8")
			return 7;
		else if (machine_name == "M9")
			return 8;
		else if (machine_name == "M10")
			return 9;
		else if (machine_name == "M11")
			return 10;
		else if (machine_name == "M12")
			return 11;
		else if (machine_name == "D1")
			return 12;
		else if (machine_name == "D2")
			return 13;
		else if (machine_name == "D3")
			return 14;
		else if (machine_name == "R1")
			return 15;

		if (machine_name == "M13")
			return 16;
		else if (machine_name == "M14")
			return 17;
		else if (machine_name == "M15")
			return 18;
		else if (machine_name == "M16")
			return 19;
		else if (machine_name == "M17")
			return 20;
		else if (machine_name == "M18")
			return 21;
		else if (machine_name == "M19")
			return 22;
		else if (machine_name == "M20")
			return 23;
		else if (machine_name == "M21")
			return 24;
		else if (machine_name == "M22")
			return 25;
		else if (machine_name == "M23")
			return 26;
		else if (machine_name == "M24")
			return 27;
		else if (machine_name == "D4")
			return 28;
		else if (machine_name == "D5")
			return 29;
		else if (machine_name == "D6")
			return 30;
		else if (machine_name == "R2")
			return 31;
		else
			throw fawkes::Exception("Unknown machine type %s", machine_name.c_str());

	default: throw fawkes::Exception("Unknown assignment/machine name");
	}
}

const char *
to_string(unsigned int machine, MachineAssignment machine_assignment)
{
	switch (machine_assignment) {
	case ASSIGNMENT_2013:
		switch (machine) {
		case 0: return "M1";
		case 1: return "M2";
		case 2: return "M3";
		case 3: return "M4";
		case 4: return "M5";
		case 5: return "M6";
		case 6: return "M7";
		case 7: return "M8";
		case 8: return "M9";
		case 9: return "M10";
		case 10: return "D1";
		case 11: return "D2";
		case 12: return "D3";
		case 13: return "TST";
		case 14: return "R1";
		case 15: return "R2";
		default: throw fawkes::Exception("Unknown machine number %u", machine);
		}

	case ASSIGNMENT_2014:
		switch (machine) {
		case 0: return "M1";
		case 1: return "M2";
		case 2: return "M3";
		case 3: return "M4";
		case 4: return "M5";
		case 5: return "M6";
		case 6: return "M7";
		case 7: return "M8";
		case 8: return "M9";
		case 9: return "M10";
		case 10: return "M11";
		case 11: return "M12";
		case 12: return "D1";
		case 13: return "D2";
		case 14: return "D3";
		case 15: return "R1";

		case 16: return "M13";
		case 17: return "M14";
		case 18: return "M15";
		case 19: return "M16";
		case 20: return "M17";
		case 21: return "M18";
		case 22: return "M19";
		case 23: return "M20";
		case 24: return "M21";
		case 25: return "M22";
		case 26: return "M23";
		case 27: return "M24";
		case 28: return "D4";
		case 29: return "D5";
		case 30: return "D6";
		case 31: return "R2";
		default: throw fawkes::Exception("Unknown machine number %u", machine);
		}

	default: throw fawkes::Exception("Unknown assignment/machine name");
	}
}

} // namespace llsf_utils
