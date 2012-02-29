/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/
#ifndef LOG_H
#define LOG_H
#include <string>

/**
* @file log.h
* 
* @class CLog
*
* @brief A log that is linked to a file handle.
*
* A CLog is a log that is linked to a file handle, log entries are appended to the file.
* Entries are added to the log using the Write method, which works in a printf fashion.
*/

class CLog
{
public:
	CLog(void);

/**
* Opens a file handle named by strFileName
*
* @param strFileName The name of the log file.
*
*/
	CLog(const char *strFileName);
	~CLog(void);

/**
* Writes an entry to the log, this function is used just like printf.
*
*/
	void Write(char *format, ...);

/**
* Writes an entry to the log, this function is used just like printf.
*
*/
  void Write(const char *format, ...);


//<member_variables>
	std::string m_sName;
//</member_variables>

};
#endif
