///////////////////////////////////////////////////////////
//  sql_manager.cpp
//  Implementation of the Class sql_manager
//  Created on:      2015/08/28 18:08:50
//  Original author: aksuzuki
///////////////////////////////////////////////////////////
#if defined (_WIN32)
#include <WinSock2.h>

#include <windows.h>
#include <windowsx.h>
#endif

#include "sql_manager.h"
#define ENABLE_ENCRYPT_DB


int sql_manager::err = 0;
char *sql_manager::sql_buf = nullptr;
#if defined (_WIN32)
const char *sql_manager::db_file = "./rayjack";// "./rayjack_crypt";
const char *sql_manager::db_key = "x'77553508d388e9ab40617d4f8c893f291faab111e7f311207c7f62607b8783c0'";
//char *sql_manager::db_file = "./db/myassets.db";
#else
char sql_manager::db_file[256];
char sql_manager::db_key[256];
#endif
sqlite3 *sql_manager::db = nullptr;
sqlite3_stmt *sql_manager::stmt = nullptr;
bool sql_manager::isStart = false;
bool sql_manager::state = false;
bool sql_manager::useMutex = false;
int  sql_manager::colum_count=0;
void sql_manager::err_box(const char* in_err_msg, const char* in_err_code)
{
#if defined (_WIN32)
	MessageBoxA(NULL, in_err_msg, in_err_code, MB_OK);
#endif
}

void sql_manager::set_db(const char *in_buf, const char* in_key)
{
#if defined (_WIN32)
	db_file = in_buf;
	db_key = in_key;
#else
	strcpy(db_file, in_buf);
	strcpy(db_key, in_key);
#endif
}

bool sql_manager::db_open()
{
	db_finalize();
	if (isStart)return true;
	bool ret = false;
	if (state)
	{
		return true;
	}
	else{
		state = true;
	}
	if (db != nullptr)
	{
		if (db != nullptr){
			//err_msg("db opend err", 0);
			return ret;
		}

	}
	if (db_file == nullptr || db_file[0] == 0)
	{
		err_msg("db file not set err", 0);
		return ret;

	}
	//sqlite3_initialize();
	setUseMutex(true);
	if (SQLITE_OK != (err = useMutex ? sqlite3_open_v2(db_file, &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX, 0) : sqlite3_open(db_file, &db)))
	{
		err_msg("open db error:", err);
		sqlite3_close(db);
		return ret;
	}
	// 暗号化キーが設定されていればキーを設定する
	if (db_key)
	{
#ifndef ENABLE_ENCRYPT_DB
		if(strstr(db_file, "crypt"))
			if (SQLITE_OK != (err = sqlite3_key(db, db_key, strlen(db_key))))
			{
				err_msg("open db error:", err);
				sqlite3_close(db);
				return ret;
			}
#endif
	}
	ret = true;
	return ret;
}

void sql_manager::db_finalize()
{
	sqlite3_reset(stmt);

	if (stmt != nullptr)sqlite3_finalize(stmt);
	stmt = nullptr;
	
	if (isStart)
	{
		return;
	}
	
	if (sql_buf != nullptr)
	{
		delete sql_buf;
		sql_buf = nullptr;
	}
	
}


bool sql_manager::db_begin_no_open(const char * in_buf, int in_cond1 /*= 0*/, int in_cond2 /*= 0*/)
{
	bool ret = false;
//	err = sqlite3_initialize();
//	if (err != SQLITE_OK)
//	{
//		err_msg(in_buf);
//		sqlite3_reset(stmt);
//		return ret;
//	}
//	else
	{
		db_finalize();
		err = sqlite3_prepare_v2(db, in_buf, -1, &stmt, NULL);
		if (err != SQLITE_OK)
		{
			
			err_msg(in_buf);
			sqlite3_reset(stmt);
			return ret;
		}
		
	}
	ret = true;
	return ret;
}

bool sql_manager::db_begin(const char * in_buf, int in_cond1 /*= 0*/, int in_cond2 /*= 0*/)
{
	bool ret = false;
	
	if (db_open())
	{
//		sqlite3_initialize();
		db_finalize();
		err = sqlite3_prepare_v2(db, in_buf, -1, &stmt, NULL);
		if (err != SQLITE_OK)
		{
			db_close();
			err_msg(in_buf);
			
			//err_msg("prepare open error", err);
			return ret;
		}
		ret = true;
	}
	
	
	return ret;
}

bool sql_manager::db_begin_file_cond(const char * in_buf,const char*in_cond, int in_cond1 /*= 0*/, int in_cond2 /*= 0*/)
{
	bool ret = false;
	/*
		size_t buf_len = strlen(in_buf);
		sql_buf = new char[buf_len + 20];
		sprintf(sql_buf, in_buf, in_cond1, in_cond2);
		*/
	sqlite3_bind_text(stmt, 1, in_cond,-1, SQLITE_TRANSIENT);
	sqlite3_bind_text(stmt, 2, in_cond,-1, SQLITE_TRANSIENT);
	
	if (db_open())
	{
		//sqlite3_initialize();
		db_finalize();
		err = sqlite3_prepare_v2(db, in_buf, -1, &stmt, NULL);
		if (err != SQLITE_OK)
		{
			db_close();
			err_msg(in_buf);
			//err_msg("prepare open error", err);
			return ret;
		}
		ret = true;
	}
	
	
	return ret;
}


int sql_manager::get_int_no_open(const char *in_buf, int in_colum_no /*= 0*/)
{
	
	int ret = 0;
//	err=sqlite3_initialize();
//	if (err != SQLITE_OK)
//	{
//		err_msg(in_buf);
//		db_close();
//		
//		return ret;
//	}

	db_finalize();
	err = sqlite3_prepare_v2(db, in_buf, -1, &stmt, NULL);
	if (err != SQLITE_OK)
	{
		err_msg(in_buf);
		db_close();
		
		return ret;
	}
	
	if (SQLITE_ROW == (err = sqlite3_step(stmt)))
	{
		ret = sqlite3_column_int(stmt, in_colum_no);
	}
	sqlite3_reset(stmt);
	
	return ret;
}

bool sql_manager::exe(const char* in_sql,int in_colum)
{
	db_finalize();
	bool result = sql_manager::db_open();
	err = sqlite3_prepare_v2(db, in_sql, -1, &stmt, NULL);
	if (err != SQLITE_OK)
	{
		
		err_msg(in_sql);
		sqlite3_reset(stmt);
		return false;
	}
	sqlite3_bind_int(stmt, 1, in_colum);
	//		err = sqlite3_exec(db, in_sql, nullptr, nullptr, nullptr);
	err = sqlite3_step(stmt);
	if (err != SQLITE_OK)
	{
		
		err_msg(in_sql);
		sql_manager::db_close();
		return false;
	}
	sql_manager::db_close();
	return true;
	
}

