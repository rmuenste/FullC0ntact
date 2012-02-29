#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include "log.h"

CLog::CLog(void)
{
}

CLog::~CLog(void)
{
}

CLog::CLog(const char *strFileName)
{
	m_sName = std::string(strFileName);
}

void CLog::Write(char *format, ...)
{
	va_list params;
	time_t t;
    struct tm *ts;

    t = time(NULL);
    ts = localtime(&t);
   	char str[80];
	strftime(str, 80, "%d.%m.%y %H:%M:%S", ts);

	FILE * fp = fopen(m_sName.c_str(), "a");
	if(fp) {
		fprintf(fp, "%s: ", str);
		va_start(params, format);
		vfprintf(fp, format, params);
		va_end(params);
		fprintf(fp, "\n");
		fprintf(fp, "-----------------------------------\n");
		fclose(fp);
	}
}

void CLog::Write(const char *format, ...)
{
	va_list params;
	time_t t;
    struct tm *ts;

    t = time(NULL);
    ts = localtime(&t);
   	char str[80];
	strftime(str, 80, "%d.%m.%y %H:%M:%S", ts);

	FILE * fp = fopen(m_sName.c_str(), "a");
	if(fp) {
		fprintf(fp, "%s: ", str);
		va_start(params, format);
		vfprintf(fp, format, params);
		va_end(params);
		fprintf(fp, "\n");
		fprintf(fp, "-----------------------------------\n");
		fclose(fp);
	}
}
