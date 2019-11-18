#ifndef VBASE_BASE_H
#define VBASE_BASE_H

#include <string.h>
#include <assert.h>
#include <string>

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifdef COMMON_EXPORT
#define COMMON_API  __declspec(dllexport)
#else 
#define COMMON_API  __declspec(dllimport)
#endif
#else	//WIN32 END
#define COMMON_API
#endif

#if !defined __GNUC_MINOR__ || defined __INTEL_COMPILER || defined __SUNPRO_C || defined __SUNPRO_CC || defined __llvm__ || defined __clang__ || defined _MSC_VER
  #define base_gcc_version(major,minor) 0
#else
  #define base_gcc_version(major,minor) (__GNUC__ > (major) || (__GNUC__ == (major) && __GNUC_MINOR__ >= (minor)))
#endif

/* memory fence defines */
#ifdef _MSC_VER
  #include <intrin.h>
  #if _MSC_VER >= 1800 /* VC++ 2013 */
	#define base_memory_fence()  _ReadWriteBarrier(); MemoryBarrier()
  #elif _MSC_VER >= 1500 /* VC++ 2008 */
  	/* apparently, microsoft broke all the memory barrier stuff in Visual Studio 2008... */
	#define base_memory_fence()  _ReadWriteBarrier(); MemoryBarrier()
  #elif _MSC_VER >= 1400 /* VC++ 2005 */
	#define base_memory_fence()  _ReadWriteBarrier()
  #elif defined _WIN32
	#include <WinNT.h>
	#define base_memory_fence()  MemoryBarrier() /* actually just xchg on x86... scary */
  #endif
#else
  #ifndef base_memory_fence
    #if base_gcc_version(2,5) || defined __INTEL_COMPILER || (__llvm__ && __GNUC__) || __SUNPRO_C >= 0x5110 || __SUNPRO_CC >= 0x5110
      #if __i386 || __i386__
        #define base_memory_fence()    		__asm__ __volatile__("lock; orb $0, -1(%%esp)" : : : "memory")
        #define base_memory_fence_acq() 	__asm__ __volatile__(""                        : : : "memory")
        #define base_memory_fence_rls() 	__asm__ __volatile__("")
      #elif __amd64 || __amd64__ || __x86_64 || __x86_64__
        #define base_memory_fence()       	__asm__ __volatile__("mfence"   : : : "memory")
        #define base_memory_fence_acq() 	__asm__ __volatile__(""         : : : "memory")
        #define base_memory_fence_rls() 	__asm__ __volatile__("")
      #elif __powerpc__ || __ppc__ || __powerpc64__ || __ppc64__
        #define base_memory_fence()      	__asm__ __volatile__("sync"     : : : "memory")
      #elif defined __ARM_ARCH_6__  || defined __ARM_ARCH_6J__  \
         || defined __ARM_ARCH_6K__ || defined __ARM_ARCH_6ZK__
        #define base_memory_fence()      	__asm__ __volatile__("mcr p15,0,%0,c7,c10,5" : : "r" (0) : "memory")
      #elif defined __ARM_ARCH_7__  || defined __ARM_ARCH_7A__  \
         || defined __ARM_ARCH_7M__ || defined __ARM_ARCH_7R__
        #define base_memory_fence()      	__asm__ __volatile__("dmb"      : : : "memory")
      #elif __aarch64__
        #define base_memory_fence()     	__asm__ __volatile__("dmb ish"  : : : "memory")
      #elif (__sparc || __sparc__) && !__sparcv8
        #define base_memory_fence()       	__asm__ __volatile__("membar #LoadStore | #LoadLoad | #StoreStore | #StoreLoad" : : : "memory")
        #define base_memory_fence_acq() 	__asm__ __volatile__("membar #LoadStore | #LoadLoad"                            : : : "memory")
        #define base_memory_fence_rls() 	__asm__ __volatile__("membar #LoadStore             | #StoreStore")
      #elif defined __s390__ || defined __s390x__
        #define base_memory_fence()      	__asm__ __volatile__("bcr 15,0" : : : "memory")
      #elif defined __mips__
        /* GNU/Linux emulates sync on mips1 architectures, so we force its use */
        /* anybody else who still uses mips1 is supposed to send in their version, with detection code. */
        #define base_memory_fence()    		__asm__ __volatile__(".set mips2; sync; .set mips0" : : : "memory")
      #elif defined __alpha__
        #define base_memory_fence()     	__asm__ __volatile__("mb"       : : : "memory")
      #elif defined __hppa__
        #define base_memory_fence()      	__asm__ __volatile__(""         : : : "memory")
        #define base_memory_fence_rls() 	__asm__ __volatile__("")
      #elif defined __ia64__
        #define base_memory_fence()       	__asm__ __volatile__("mf"       : : : "memory")
      #elif defined __m68k__
        #define base_memory_fence()      	__asm__ __volatile__(""         : : : "memory")
      #elif defined __m88k__
        #define base_memory_fence()     	__asm__ __volatile__("tb1 0,%%r0,128" : : : "memory")
      #elif defined __sh__
        #define base_memory_fence()     	__asm__ __volatile__(""         : : : "memory")
      #endif
    #endif
  #endif

  #ifndef base_memory_fence
	#if base_gcc_version(4,7)
      /* see comment below (stdatomic.h) about the C11 memory model. */
      #define base_memory_fence()       __atomic_thread_fence(__ATOMIC_SEQ_CST)
      #define base_memory_fence_acq() 	__atomic_thread_fence(__ATOMIC_ACQUIRE)
      #define base_memory_fence_rls() 	__atomic_thread_fence(__ATOMIC_RELEASE)
  	#elif ECB_GCC_VERSION(4,4) || defined __INTEL_COMPILER || defined __clang__
      #define base_memory_fence()      	__sync_synchronize()
	#elif __SUNPRO_C >= 0x5110 || __SUNPRO_CC >= 0x5110
      #include <mbarrier.h>
      #define base_memory_fence()       __machine_rw_barrier()
      #define base_memory_fence_acq() 	__machine_r_barrier()
      #define base_memory_fence_rls() 	__machine_w_barrier()
  	#elif __xlC__
      #define base_memory_fence()       __sync ()
    #endif
  #endif
#endif

/* atomic operations defines */
#if defined(_MSC_VER)
  #include <Windows.h>
  #define base_fetch_and_inc(x) InterlockedExchangeAdd((x), 1)
  #define base_fetch_and_dec(x) InterlockedExchangeAdd((x), -1)
  #define base_fetch_and_add(x, c) InterlockedExchangeAdd((x), c)
  #define base_fetch_and_sub(x, c) InterlockedExchangeAdd((x), -(c))
  #define base_compare_and_exchange(des, cmp, exch) InterlockedCompareExchange((des), (exch), (cmp))
  #define base_exchange(des, exch) InterlockedExchange((des), (exch))
#else
  #define base_fetch_and_inc(x) __sync_fetch_and_add((x), 1)
  #define base_fetch_and_dec(x) __sync_fetch_and_sub((x), 1)
  #define base_fetch_and_add(x, c) __sync_fetch_and_add((x), c)
  #define base_fetch_and_sub(x, c) __sync_fetch_and_sub((x), c)
  #define base_compare_and_exchange(des, cmp, exch) __sync_val_compare_and_swap((des), (cmp), (exch))
  #define base_exchange(des, exch) __sync_lock_test_and_set((des), (exch))
#endif

/****************************************************************************/

#if __GNUC__>=4
#ifdef __linux__
	#include <tr1/unordered_map>
	#define VBASE_HASH_MAP std::tr1::unordered_map
#else
	#include <unordered_map>
	#define VBASE_HASH_MAP std::unordered_map
#endif
#else
	#include <unordered_map>
	#define VBASE_HASH_MAP std::unordered_map
#endif

class COMMON_API string_hash {
public:
	size_t operator()(const char* str) const {
		size_t hash = 1315423911;
		while (*str){
			hash ^= ((hash << 5) + (*str++) + (hash >> 2));
		}
		return (hash & 0x7FFFFFFF);
	}
};

class COMMON_API string_compare {
public:
	bool operator()(const char* a, const char* b) const {
		return (strcmp(a, b) == 0);
	}
};

#define CHECK_LABEL(ret, label) { if (ret != 0) goto label; }
#define GOTO_LABEL(label) { goto label; }
#define BSUCCEEDED(ret) (ret == 0)
#define BFAILED(ret) (ret != 0)
#define ASSIGN_AND_CHECK_LABEL(ret, val, label) { ret = val; if (ret != 0) goto label; }
#define LABEL_SCOPE_START {
#define LABEL_SCOPE_END }
#define SAFE_RELEASE(x) { if (x != NULL) { delete x; x = NULL; } }
#define SAFE_RELEASE_EX(x, op) { if (x != NULL) { x->op(); delete x; x = NULL; } }
#define BMAX(a,b) ((a) > (b) ? (a) : (b))
#define BMIN(a,b) ((a) < (b) ? (a) : (b))
#define BABS(a) ((a) > 0 ? (a) : -(a))
#define FEPSILON 0.000001	/* 1e-6*/
#define DEPSILON 0.000000000000001 /* 1e-15 */
#define IS_FVALUE_EQUAL(a, b) ((BABS(a) >= 1 && BABS((a) - (b)) <= FEPSILON * BABS(a)) || (BABS((a) - (b)) <= FEPSILON))
#define IS_DVALUE_EQUAL(a, b) ((BABS(a) >= 1 && BABS((a) - (b)) <= DEPSILON * BABS(a)) || (BABS((a) - (b)) <= DEPSILON))
#define COMPARE_VALUE(a, b)  ((a) > (b) ? 1 : (a) == (b) ? 0 : -1)
#define COMPARE_FVALUE(a, b) (IS_FVALUE_EQUAL((a), (b)) ? 0 : (a) > (b) ? 1 : -1)
#define COMPARE_DVALUE(a, b) (IS_DVALUE_EQUAL((a), (b)) ? 0 : (a) > (b) ? 1 : -1)

/****************************************************************************/

namespace base {

#if defined(_MSC_VER)
#ifdef _WIN32
	typedef unsigned __int64 bint64;
#else
	typedef long bint64;
	typedef unsigned long ubint64;
#endif
#else
#if __WORDSIZE == 64
	typedef long bint64;
#else
	typedef long long bint64;
#endif
#endif

#if defined(__APPLE__)

	#define PLATFORM_APPLE
	#define PLATFORM_POSIX
	#define OS64

	typedef int8_t BOOLEAN;
	typedef int8_t int8;
	typedef int16_t int16;
	typedef int32_t int32;
	typedef long int64;
	typedef long date;
	typedef float float32;
	typedef double float64;

	typedef uint8_t uint8;
	typedef uint16_t uint16;
	typedef uint32_t uint32;
	typedef unsigned long uint64;
	typedef uint8_t byte;
	typedef uint8_t size8;
	typedef uint16_t size16;
	typedef uint32_t size32;
	typedef unsigned long size64;

#elif defined(__linux__)

	#define PLATFORM_LINUX
	#define PLATFORM_POSIX
	#define OS64

	typedef char BOOLEAN;
	typedef signed char int8;
	typedef short int16;
	typedef int int32;
#if __WORDSIZE == 64
	typedef long int64;
#else
	typedef long long int64;
#endif
	typedef long date;
	typedef float float32;
	typedef double float64;

	typedef unsigned char uint8;
	typedef unsigned short uint16;
	typedef unsigned int uint32;
#if __WORDSIZE == 64
	typedef unsigned long uint64;
#else
	typedef unsigned long long uint64;
#endif
#ifndef byte
	typedef unsigned char byte;
#endif
	typedef unsigned char size8;
	typedef unsigned short size16;
	typedef unsigned int size32;
#if __WORDSIZE == 64
	typedef unsigned long size64;
#else
	typedef unsigned long long size64;
#endif

#else

	#define PLATFORM_WINDOWS

	typedef __int8 BOOLEAN;
	typedef __int8 int8;
	typedef __int16 int16;
	typedef __int32 int32;
	typedef __int64 int64;
	typedef int64 date;
	typedef float float32;
	typedef double float64;

	typedef unsigned __int8 uint8;
	typedef unsigned __int16 uint16;
	typedef unsigned __int32 uint32;
	typedef unsigned __int64 uint64;
	typedef unsigned char byte;
	typedef unsigned __int8 size8;
	typedef unsigned __int16 size16;
	typedef unsigned __int32 size32;
	typedef unsigned __int64 size64;

#endif

	typedef int8 offset8;
	typedef int16 offset16;
	typedef int32 offset32;
	typedef int64 offset64;

#ifdef _WIN32
	#define OS32
	typedef offset32 offset;
	typedef size32 size_t;
#else
	#define OS64
	typedef offset64 offset;
	typedef size64 size_t;
#endif

static const int8 kMinInt8 = -128;
static const int8 kMaxInt8 = 127;
static const uint8 kMaxUInt8 = 255U;

static const int16 kMinInt16 = -32768;
static const int16 kMaxInt16 = 32767;
static const uint16 kMaxUInt16 = 65535U;

static const int32 kMinInt32 = -2147483647 - 1;
static const int32 kMaxInt32 = 2147483647;
static const uint32 kMaxUInt32 = 4294967295U;

static const int64 kMinInt64 = -9223372036854775807LL - 1;
static const int64 kMaxInt64 = 9223372036854775807LL;	
static const uint64 kMaxUInt64 = 18446744073709551615ULL;

} // end of namespace

#endif
