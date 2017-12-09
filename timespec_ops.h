#ifndef TIMESPEC_OPS_H
#define TIMESPEC_OPS_H

/*
 * macros stolen from kernel code
 */
/* Operations on timespecs */
#define	timespecclear(tvp)	((tvp)->tv_sec = (tvp)->tv_nsec = 0)
#define	timespecisset(tvp)	((tvp)->tv_sec || (tvp)->tv_nsec)
#define	timespeccmp(tvp, uvp, cmp)					\
	(((tvp)->tv_sec == (uvp)->tv_sec) ?				\
	    ((tvp)->tv_nsec cmp (uvp)->tv_nsec) :			\
	    ((tvp)->tv_sec cmp (uvp)->tv_sec))
#define	timespecadd(vvp, uvp)						\
	do {								\
		(vvp)->tv_sec += (uvp)->tv_sec;				\
		(vvp)->tv_nsec += (uvp)->tv_nsec;			\
		if ((vvp)->tv_nsec >= 1000000000) {			\
			(vvp)->tv_sec++;				\
			(vvp)->tv_nsec -= 1000000000;			\
		}							\
		if ((vvp)->tv_nsec < 0 && (vvp)->tv_sec > 0) {		\
			(vvp)->tv_sec--;				\
			(vvp)->tv_nsec += 1000000000;			\
		}							\
	} while (0)
#define	timespecsub(vvp, uvp)						\
	do {								\
		(vvp)->tv_sec -= (uvp)->tv_sec;				\
		(vvp)->tv_nsec -= (uvp)->tv_nsec;			\
		if ((vvp)->tv_nsec >= 1000000000) {			\
			(vvp)->tv_sec++;				\
			(vvp)->tv_nsec -= 1000000000;			\
		}							\
		if ((vvp)->tv_nsec < 0 && (vvp)->tv_sec > 0) {		\
			(vvp)->tv_sec--;				\
			(vvp)->tv_nsec += 1000000000;			\
		}							\
	} while (0)

static void __inline timespec_add(struct timespec *res, const struct timespec *a1, const struct timespec *a2)
{
	*res = *a1;
	timespecadd(res, a2);
}

static void __inline timespec_sub(struct timespec *res, const struct timespec *a1, const struct timespec *a2)
{
	*res = *a1;
	timespecsub(res, a2);
}

static int __inline timespec_eq(const struct timespec *a1, const struct timespec *a2)
{
	return timespeccmp(a1, a2, ==);
}

static int __inline timespec_gt(const struct timespec *a1, const struct timespec *a2)
{
	return timespeccmp(a1, a2, >);
}

static int __inline timespec_gte(const struct timespec *a1, const struct timespec *a2)
{
	return timespeccmp(a1, a2, >=);
}

static int __inline timespec_gte0(const struct timespec *a1)
{
	struct timespec zero = { 0L, 0L };
	return timespec_eq(a1, &zero);
}

#endif
