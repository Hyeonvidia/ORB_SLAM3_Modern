/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORB_SLAM3_VERBOSE_HPP
#define ORB_SLAM3_VERBOSE_HPP

#include <iostream>
#include <string>

namespace ORB_SLAM3
{

// Global log-level gate (extracted from System.hpp in P2-3; semantics
// unchanged — System's constructor still forces VERBOSITY_QUIET, an
// upstream hardcode kept for behavior parity).
class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            std::cout << str << std::endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

}

#endif // ORB_SLAM3_VERBOSE_HPP
