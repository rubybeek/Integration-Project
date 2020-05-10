/* MA02AD.f -- translated by f2c (version 20041007).
   You must link the resulting object file with libf2c:
	on Microsoft Windows system, link with libf2c.lib;
	on Linux or Unix systems, link with .../path/to/libf2c.a -lm
	or, if you install libf2c.a in a standard place, with -lf2c -lm
	-- in that order, at the end of the command line, as in
		cc *.o -lf2c -lm
	Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

		http://www.netlib.org/f2c/libf2c.zip
*/

#include "f2c.h"
#include "slicot.h"

/* Subroutine */ int ma02ad(char *job, integer *m, integer *n, doublereal *a,
	 integer *lda, doublereal *b, integer *ldb)
{
    /* System generated locals */
    integer a_dim1, a_offset, b_dim1, b_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j;


/*     RELEASE 4.0, WGS COPYRIGHT 1999. */

/*     PURPOSE */

/*     To transpose all or part of a two-dimensional matrix A into */
/*     another matrix B. */

/*     ARGUMENTS */

/*     Mode Parameters */

/*     JOB     CHARACTER*1 */
/*             Specifies the part of the matrix A to be transposed into B */
/*             as follows: */
/*             = 'U': Upper triangular part; */
/*             = 'L': Lower triangular part; */
/*             Otherwise:  All of the matrix A. */

/*     Input/Output Parameters */

/*     M      (input) INTEGER */
/*            The number of rows of the matrix A.  M >= 0. */

/*     N      (input) INTEGER */
/*            The number of columns of the matrix A.  N >= 0. */

/*     A      (input) DOUBLE PRECISION array, dimension (LDA,N) */
/*            The m-by-n matrix A.  If JOB = 'U', only the upper */
/*            triangle or trapezoid is accessed; if JOB = 'L', only the */
/*            lower triangle or trapezoid is accessed. */

/*     LDA    INTEGER */
/*            The leading dimension of the array A.  LDA >= max(1,M). */

/*     B      (output) DOUBLE PRECISION array, dimension (LDB,M) */
/*            B = A' in the locations specified by JOB. */

/*     LDB    INTEGER */
/*            The leading dimension of the array B.  LDB >= max(1,N). */

/*     CONTRIBUTOR */

/*     A. Varga, German Aerospace Center, */
/*     DLR Oberpfaffenhofen, March 1998. */
/*     Based on the RASP routine DMTRA. */

/*     REVISIONS */

/*     - */

/*     ****************************************************************** */

/*     .. Scalar Arguments .. */
/*     .. Array Arguments .. */
/*     .. Local Scalars .. */
/*     .. External Functions .. */
/*     .. Intrinsic Functions .. */

/*     .. Executable Statements .. */

    /* Parameter adjustments */
    a_dim1 = *lda;
    a_offset = 1 + a_dim1;
    a -= a_offset;
    b_dim1 = *ldb;
    b_offset = 1 + b_dim1;
    b -= b_offset;

    /* Function Body */
    if (lsame(job, "U")) {
	i__1 = *n;
	for (j = 1; j <= i__1; ++j) {
	    i__2 = min(j,*m);
	    for (i__ = 1; i__ <= i__2; ++i__) {
		b[j + i__ * b_dim1] = a[i__ + j * a_dim1];
/* L10: */
	    }
/* L20: */
	}
    } else if (lsame(job, "L")) {
	i__1 = *n;
	for (j = 1; j <= i__1; ++j) {
	    i__2 = *m;
	    for (i__ = j; i__ <= i__2; ++i__) {
		b[j + i__ * b_dim1] = a[i__ + j * a_dim1];
/* L30: */
	    }
/* L40: */
	}
    } else {
	i__1 = *n;
	for (j = 1; j <= i__1; ++j) {
	    i__2 = *m;
	    for (i__ = 1; i__ <= i__2; ++i__) {
		b[j + i__ * b_dim1] = a[i__ + j * a_dim1];
/* L50: */
	    }
/* L60: */
	}
    }

    return 0;
/* *** Last line of MA02AD *** */
} /* ma02ad_ */

