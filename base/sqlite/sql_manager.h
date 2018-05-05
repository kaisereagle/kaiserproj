///////////////////////////////////////////////////////////
//  sql_manager.h
//  Implementation of the Class sql_manager
//  Created on:      2015/08/28 18:08:50
//  Original author: aksuzuki
///////////////////////////////////////////////////////////

#if !defined(EA_DE1C91E2_C7E7_4bd7_8AC6_6C8125A85D43__INCLUDED_)
#define EA_DE1C91E2_C7E7_4bd7_8AC6_6C8125A85D43__INCLUDED_
#include <sqlite3.h>
#include <string.h>
#include <stdio.h>
class sql_manager
{
public:

	inline sql_manager()
	{

	}

	static char *new_char(const char *buf)
	{
		char* ret = NULL;
		if (buf == NULL) {
			return NULL;
		}
		size_t word_len = strlen(buf) + 1;
		ret = new char[word_len]();
		strcpy(ret, buf);
		return ret;

	}
	static char *prepare_where_cond(const char *in_sql, const char *in_cond)
	{
		size_t word_len = strlen(in_sql) + strlen(in_cond) + 1;
		char *ret = new char[word_len]();
		strcpy(ret, in_sql);
		strcat(ret, in_cond);
		return ret;


	}
	static void err_box(const char* in_err_msg, const char* in_err_code);
	static void err_msg(const char *in_msg, int in_err_no = 0)
	{

		err_box(in_msg, (const char*)sqlite3_errmsg(db));
//		delete err_msg_buf;
	}

	static void set_db(const char *in_buf, const char* in_key);

	static void db_use_start()
	{
		if (isStart)return;
		if (db_open())
		{
			isStart = true;
		}
	}

	static void db_use_end()
	{
		if (isStart)
		{
			isStart = false;
			db_end();
		}
	}

	/**
	 * SQLiteのmutexを使うか設定
	 * openする前に設定しておくこと
	 */
	static void setUseMutex(bool use)
	{
		useMutex = use;
	}

	static bool db_open();

	static void db_close()
	{
		if (isStart)return;
		if (db != nullptr)
			sqlite3_close(db);
		db = nullptr;
		state = false;

	}
	static void db_reset_stmt()
	{
		sqlite3_reset(stmt);
	}
	
	static void db_finalize();
	
	static bool db_begin_no_open(const char * in_buf, int in_cond1 = 0, int in_cond2 = 0);

	static bool db_begin(const char * in_buf, int in_cond1 = 0, int in_cond2 = 0);

	static bool db_begin_file_cond(const char * in_buf,const char*in_cond, int in_cond1 = 0, int in_cond2 = 0);

	static void db_end()
	{
		db_finalize();
		db_close();
	}
	
	static int get_int_no_open(const char *in_buf, int in_colum_no = 0);

	static int get_int(const char *in_buf, int in_colum_no = 0)
	{

		int ret = 0;

		if (db_begin(in_buf))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_int(stmt, in_colum_no);
			}
		}
		db_end();
		return ret;
	}

	static void* get_blob(const char *in_buf, int in_colum_no = 0)
	{
		void* ret=nullptr;
		if (db_begin(in_buf))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = (void*)sqlite3_column_blob(stmt, in_colum_no);
			}
		}
		//db_end();
		return ret;
	}
	static void* get_blob_file_trim(const char *in_buf, const char *in_file_name, int in_colum_no = 0)
	{
		void* ret = nullptr;
		if (db_begin_file_cond(prepare_where_cond(in_buf, in_file_name), in_file_name))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = (void*)sqlite3_column_blob(stmt, in_colum_no);
			}
		}
		//db_end();
		return ret;
	}
	static int get_bytes(const char *in_buf, int in_colum_no = 0)
	{
		int ret=0;
		if (db_begin(in_buf))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_bytes(stmt, in_colum_no);
			}
		}
		db_end();
		return ret;
	}
	static int get_byte_file_trim(const char *in_buf,const char *in_file_name, int in_colum_no = 0)
	{
		int ret = 0;

		if (db_begin_file_cond(prepare_where_cond(in_buf, in_file_name), in_file_name))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_bytes(stmt, in_colum_no);
			}
		}
		db_end();
		return ret;
	}
	static const unsigned char* get_text(const char *in_buf, int in_colum_no = 0)
	{
		const unsigned char* ret=nullptr;
		if (db_begin(in_buf))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_text(stmt, in_colum_no);
			}
		}
		//db_end();
		return ret;
	}
	static const unsigned char* get_text(const char *in_buf, const char* in_param)
	{
		const unsigned char* ret = nullptr;
		if (db_begin(in_buf))
		{
			int colum_no = field_index(in_param);
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_text(stmt, colum_no);
			}
		}
		//db_end();
		return ret;
	}
	static int field_index(const char* in_colum)
	{
		colum_count = sqlite3_column_count(stmt);
		if (in_colum)
		{
			for (int i = 0; i < colum_count; i++)
			{
				const char* tmp = sqlite3_column_name(stmt, i);

				if (strcmp(in_colum, tmp) == 0)
				{
					return i;
				}
			}
		}
		return -1;
	}

	static int get_int_cond(const char *in_buf, int in_colum_no = 0)
	{
		int ret=0;
		if (db_begin(in_buf,0,0))
		{
			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_int(stmt, in_colum_no);
			}
		}
//		db_end();
		return ret;
	}
	static int get_stap_int(int in_colum)
	{
		return sqlite3_column_int(stmt, in_colum);
	}
	static sqlite3_int64 get_stap_int64(int in_colum)
	{
		return sqlite3_column_int64(stmt, in_colum);
	}
	static float get_stap_float(int in_colum)
	{
		return (float)sqlite3_column_double(stmt, in_colum);
	}
	static char* get_stap_text(int in_colum)
	{
		return new_char((char *)sqlite3_column_text(stmt, in_colum));
	}
	static int get_colum_count()
	{
		return sqlite3_column_count(stmt);
	}

	static bool begin_transaction()
	{
		return true;
		//return exec("BEGIN;");
	}

	static bool commit_transaction()
	{
		return true;
	//	return exec("COMMIT;");
	}

	static bool rollback_transaction()
	{
		return exec("ROLLBACK;");
	}

	static bool exec(const char* in_sql, char** msg = nullptr, int* errp = nullptr)
	{
		err = sqlite3_exec(db,in_sql,nullptr,nullptr,msg);
		if (errp) *errp = err;
		return (err == SQLITE_OK);
	}
	static bool exe_condition(const char* in_sql,int in_colum)
	{
		bool result = sql_manager::db_open();

		err = sqlite3_exec(db, in_sql, nullptr, nullptr, nullptr);
		if (err != SQLITE_OK)
		{

			err_msg(in_sql);
			sql_manager::db_close();
			return false;
		}
		sql_manager::db_close();
		return true;

	}
	static bool exe(const char* in_sql)
	{
		bool result = sql_manager::db_open();
		err = sqlite3_exec(db, in_sql, nullptr, nullptr, nullptr);
		if (err != SQLITE_OK)
		{

			err_msg(in_sql);
			sql_manager::db_close();
			return false;
		}
		sql_manager::db_close();
		return true;

	}
	static bool exe(const char* in_sql,int in_colum);
	
	static int get_table_count()
	{
		int count=0;
		while (sqlite3_step(stmt) == SQLITE_DONE)
		{
			count = sqlite3_column_int(stmt, 0);
		}

		return count;
	}
	static int step()	//	stepのみ
	{
		return sqlite3_step(stmt);
	}

	static int bind_stap_text(const char* text, int index)
	{
		int ret;
		ret = sqlite3_bind_text(stmt, index, text, -1, SQLITE_STATIC);

		return ret;
	}

	static bool vacuum() {
		return exec("VACUUM;");
	}

	static const unsigned char* get_text_cond(const char *in_buf, const char* in_param, const int num_cond, ...)
	{
		va_list list;

		va_start(list, num_cond);

		const unsigned char* ret = nullptr;

		if (db_begin(in_buf))
		{
			// 渡したパラメータをセット(全てint固定)
			for (int n = 0; n < num_cond; n++)
			{
				const int cond = va_arg(list, int);

				sqlite3_bind_int(stmt, n + 1, cond);
			}

			const int colum_no = field_index(in_param);

			if (SQLITE_ROW == (err = sqlite3_step(stmt)))
			{
				ret = sqlite3_column_text(stmt, colum_no);
			}
		}
		//db_end();

		va_end(list);

		return ret;
	}

	static bool set_cond(const char *in_buf, const int num_cond, ...)
	{
		va_list list;

		va_start(list, num_cond);

		bool ret = db_begin(in_buf);

		if (ret)
		{
			// 渡したパラメータをセット(全てint固定)
			for (int n = 0; n < num_cond; n++)
			{
				const int cond = va_arg(list, int);

				sqlite3_bind_int(stmt, n + 1, cond);
			}
		}
		va_end(list);

		return ret;
	}

	static const char* get_db_file() { return db_file; }
	static const char* get_db_key() { return db_key; }

private:
	static	sqlite3_stmt* stmt;
	static	sqlite3* db;
#if defined (_WIN32)
	static	const char* db_file;
	static	const char* db_key;
#else
	static	char db_file[256];
	static	char db_key[256];
#endif
	static	int err;
	static char *sql_buf;
	static bool isStart;
	static bool state;
	static bool useMutex;
	static int colum_count;
};
#endif // !defined(EA_DE1C91E2_C7E7_4bd7_8AC6_6C8125A85D43__INCLUDED_)
