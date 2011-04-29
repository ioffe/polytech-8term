
#pragma once

#include <io.h>

#include < string    >
#include < algorithm >
#include < deque     >

#include "safe_bool.h"


#ifdef UNICODE
#  define string_type std::wstring
#else
#  define string_type std::string
#endif


namespace FileUtils
{

//
//
struct File
{
    File( LPCTSTR name, LPCTSTR path ) : name_( name ), path_( path ){}
    LPCTSTR name() const { return name_.c_str(); }
    LPCTSTR path() const { return path_.c_str(); }
    operator LPCTSTR () const { return path();   }
private:
    string_type name_;
    string_type path_;
};

struct CompareFileNames
{
    CompareFileNames( LPCTSTR name ) : name_( name ) {}

    bool operator() ( const File& f ) const
    {
        return ( 0 == _tcsicmp( name_, f.name() ) );
    }

private:
    LPCTSTR name_;
};

struct CompareFilePaths
{
    CompareFilePaths( LPCTSTR path ) : path_( path ) {}

    bool operator() ( const File& f ) const
    {
        return ( 0 == _tcsicmp( path_, f.path() ) );
    }

private:
    LPCTSTR path_;
};

struct Files
{
    typedef std::deque< File >::iterator       iterator;
    typedef std::deque< File >::const_iterator const_iterator;

    iterator begin() { return files.begin(); }
    iterator end()   { return files.end();   }

    const_iterator begin() const { return files.begin(); }
    const_iterator end()   const { return files.end();   }

    const_iterator find_name ( LPCTSTR name ) const
    {
        return std::find_if( begin(), end(), CompareFileNames( name ) );
    }

    const_iterator find_path ( LPCTSTR path ) const
    {
        return std::find_if( begin(), end(), CompareFilePaths( path ) );
    }

    DWORD size() const { return files.size(); }

protected:
    //void push_back( const File& f ) { files.push_back( f ); }
    void push_back( LPCTSTR path )
    { 
        files.push_back( File( PathFindFileName( path ), path ) ); 
    }

private:
    std::deque< File > files;
};

//
//
struct ScanFolder : Files
{
    ScanFolder ( LPCTSTR root = 0, LPCTSTR mask = 0, bool recursive = true) : 
        root_( root ? root : _T("") ), mask_( mask ? mask : _T("*.*") ), recursive_(recursive)
    {
    }

    ScanFolder ( const ScanFolder& a )
    {
        static_cast< Files& >(*this) = static_cast< const Files& >( a );

        root_      = a.root_;
        mask_      = a.mask_;
        recursive_ = a.recursive_;
    }

    void operator() ( const string_type& folder )
    {
        (*this)( folder.c_str() );
    }

    void operator() ( LPCTSTR folder = 0 )
    {
        if ( !PathIsDirectory( folder ) && PathFileExists( folder ) )
        {
          push_back( folder );
          return;
        }


        string_type root = folder ? folder : root_;     
        if ( *root.rbegin() != _T('\\') ) root.append( _T("\\") );
        string_type mask( root );
        //mask.append( mask_ );
        mask.append( _T("*.*") );

        // Find first file
        _tfinddata_t find_file;
        long file = _tfindfirst( const_cast<LPTSTR>(mask.c_str()), &find_file );
        if ( file == -1L ) return;

        // Find the rest of the files
        do
        {
            if ( isDots( find_file ) ) continue;

            string_type path( root );
            path.append( find_file.name );

            if ( isDirectory( find_file ) ) 
            {
                if ( recursive_ ) (*this)( path.c_str() );
            }
            else
            {
                if ( PathMatchSpec( find_file.name, mask_.c_str() ) ) push_back( path.c_str() );
            }

        } while( _tfindnext( file, &find_file ) == 0 );

        _findclose( file );
    }

public:
    static void DeleteFolderContent( LPCTSTR path );

private:
    static bool isDots( LPCTSTR path )
    {
        if ( 0 == *path || 0 == path )
           return false;
        if ( path[0] == _T('.') )
            if ( path[1] == 0 || ( path[1] == _T('.') && path[2] == 0 ) )
                return true;
        return false;
    }
    static bool isDots     ( const struct _tfinddata_t& f ) { return isDots( f.name ); }
    static bool isDirectory( const struct _tfinddata_t& f ) { return ( ( f.attrib & _A_SUBDIR ) && !isDots( f ) ); }

private:
    string_type root_;
    string_type mask_;
    bool        recursive_;

};

//
//
struct Copy : Files
{
    Copy( LPCTSTR dst ) : dst_( dst ), err_(false) {  }
    
    void operator()( const File& file )
    {
        PathCombine( buff_, dst_.c_str(), file.name() );

        if ( CopyFile( file.path(), buff_, FALSE ) > 0 )
        {
            push_back( buff_ );
        }
        else
        {
            err_ = true;
        }
    }

    SAFE_BOOL_OPERATOR(err_)

private:
    string_type dst_;    
    TCHAR buff_[MAX_PATH];
    bool err_;
};

//
//
struct Delete : Files
{
    Delete() : err_( false ) {}

    void operator()( LPCTSTR path )
    {
        if ( DeleteFile( path ) > 0 )
        {
            push_back( path );
        }
        else
        {
            err_ = true;
        }
    }

    SAFE_BOOL_OPERATOR(err_)

private:
    bool err_;
};

inline void ScanFolder::DeleteFolderContent( LPCTSTR path )
{
    if ( !PathFileExists( path ) || !PathIsDirectory( path ) )
        return;


    string_type root = path;     
    if ( *root.rbegin() != _T('\\') ) root.append( _T("\\") );

    string_type mask( root );
    mask.append( _T("*.*") );

    // Find first file
    _tfinddata_t find_file;
    long file = _tfindfirst( const_cast<LPTSTR>(mask.c_str()), &find_file );
    if ( file == -1L ) return;

    // Find the rest of the files
    do
    {
        if ( isDots( find_file ) ) continue;

        string_type path( root );
        path.append( find_file.name );

        if ( isDirectory( find_file ) ) 
            DeleteFolderContent( path.c_str() );            
        else
            DeleteFile( path.c_str() );

    } while( _tfindnext( file, &find_file ) == 0 );

    _findclose( file );
}

};

#undef string_type
