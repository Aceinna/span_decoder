#ifdef MINIZIPDLL_EXPORTS
#define MINIZIPDLL_API __declspec(dllexport)
#else
#define MINIZIPDLL_API __declspec(dllimport)
#endif

// export
class MINIZIPDLL_API Cminizipdll {
public:
	Cminizipdll(void);
	// TODO: add methods here
};

extern MINIZIPDLL_API int nminizipdll;

MINIZIPDLL_API int fnminizipdll(void);

MINIZIPDLL_API int Add(int a, int b);

MINIZIPDLL_API int minizip(int argc, char* argv[]);

