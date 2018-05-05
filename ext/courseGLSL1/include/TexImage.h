
#ifdef LIBIMAGE_EXPORTS
#define IMAGE_API __declspec(dllexport)
#else
#define IMAGE_API __declspec(dllimport)
#endif

class IMAGE_API CTexImage
{
public:
	CTexImage(void);

public:
	unsigned char *m_buffer;
	unsigned int m_width, m_height;

public:
	bool ReadFile(const char *filename);

public:
	~CTexImage(void);
};
