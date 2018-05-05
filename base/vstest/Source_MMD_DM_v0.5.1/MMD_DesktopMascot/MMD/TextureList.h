//**********************
// テクスチャリスト管理
//**********************

#ifndef	_TEXTURELIST_H_
#define	_TEXTURELIST_H_

class cTextureList
{
	private :
		struct TextureData
		{
			unsigned int	uiTexID;
			unsigned int	uiRefCount;

			TextureData		*pNext;

			char			szFileName[32];
		};

		TextureData		*m_pTextureList;

		bool findTexture( const char *szFileName, unsigned int *puiTexID );

		bool createTexture( const char *szFileName, unsigned int *puiTexID );
		bool createFromBMP( const unsigned char *pData, unsigned int *puiTexID );
		bool createFromTGA( const unsigned char *pData, unsigned int *puiTexID );

		// 2009/07/09 Ru--en
		int  ceilToPowerOfTwo( int number );
		void fitTextureSize( unsigned char **pData, int *width, int *height ); 

		// 2011/04/30 Ru--en
#ifdef _WIN32
		bool createUsingGdi( const WCHAR *wszFileName, unsigned int *puiTexID );
#endif

	public :
		cTextureList( void );
		~cTextureList( void );

		unsigned int getTexture( const char *szFileName );

		void debugDraw( void );

		void releaseTexture( unsigned int uiTexID );
};

#endif	// _TEXTURELIST_H_
