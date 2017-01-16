/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Useful pdf functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_pdf.c 6348 2008-04-17 02:53:17Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>

#include "pf_pdf.h"
#include <ros/ros.h>
// Random number generator seed value
static unsigned int pf_pdf_seed;


/**************************************************************************
* Uniform Distribution
* add by Bill.xie xiekun@cvte.com
**************************************************************************/
bool onSegment(pf_vector_t p, pf_vector_t q, pf_vector_t r)
{
    if(q.v[0] <= std::max(p.v[0], r.v[0]) && 
        q.v[0] >= std::min(p.v[0], r.v[0]) &&
        q.v[1] <= std::max(p.v[1], r.v[1]) &&
        q.v[1] <= std::min(p.v[1], r.v[1]))
        return true;
    return false;
}

int orientation(pf_vector_t p, pf_vector_t q, pf_vector_t r)
{
    int val = (q.v[1] - p.v[1]) * (r.v[0] - q.v[0]) -
              (q.v[0] - p.v[0]) * (r.v[1] - q.v[1]);
    if (val == 0) return 0;
    return (val > 0) ? 1 : 2;
}


bool doIntersect(pf_vector_t p1, pf_vector_t p2, pf_vector_t tp1, pf_vector_t tp2)
{
    int o1 = orientation(p1, p2, tp1);
    int o2 = orientation(p1, p2, tp2);
    int o3 = orientation(tp1, tp2, p1);
    int o4 = orientation(tp1, tp2, p2);
    if (o1 != o2 && o3 != o4)
        return true;
    if (o1 == 0 && onSegment(p1, tp1, p2)) return true;
    if (o2 == 0 && onSegment(p1, tp2, p2)) return true;
    if (o3 == 0 && onSegment(tp1, p1, tp2)) return true;
    if (o4 == 0 && onSegment(tp1, p2, tp2)) return true;
    return false;
}
// http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon
int contains(pf_pdf_uniform_t* pdf, pf_vector_t test_point)
{
  int crossings = 0;
  int i;
  pf_vector_t extreme_point;
  extreme_point.v[0] = 60000;
  extreme_point.v[1] = test_point.v[1];
  extreme_point.v[2] = 0;
  for (i = 0; i < pdf->polygon.size; ++i)
  {
    pf_vector_t point = pdf->polygon.points[i];
    pf_vector_t next_point = pdf->polygon.points[(i + 1) % pdf->polygon.size];
    if (doIntersect(point, next_point, test_point, extreme_point))
    {
       if(orientation(point, test_point, next_point) == 0)
        {
            return onSegment(point, test_point, next_point);
        }
        crossings++;
    }
  }
  return crossings % 2 != 0;
}


// ignore the v[2] dimention.
pf_pdf_uniform_t *pf_pdf_uniform_alloc(pf_polygon_t polygon)
{
  if (polygon.size < 3)
    return NULL;
  pf_pdf_uniform_t* pdf = (pf_pdf_uniform_t*)malloc(sizeof(pf_pdf_uniform_t));
  pdf->polygon.size = polygon.size;
  pdf->polygon.points = (pf_vector_t*)malloc(sizeof(pf_vector_t) * polygon.size);
  pdf->min_x = pdf->min_y = 65500;
  pdf->max_x = pdf->max_y = -65500;
  int i;
  for (i = 0; i < polygon.size; ++i)
  {
    pdf->polygon.points[i] = polygon.points[i];
    if (polygon.points[i].v[0] > pdf->max_x)
    {
      pdf->max_x = polygon.points[i].v[0];
    }
    if (polygon.points[i].v[0] < pdf->min_x)
    {
      pdf->min_x = polygon.points[i].v[0];
    }
    if (polygon.points[i].v[0] > pdf->max_y)
    {
      pdf->max_y = polygon.points[i].v[1];
    }
    if (polygon.points[i].v[0] < pdf->min_y)
    {
      pdf->min_y = polygon.points[i].v[1];
    }
  }
  srand48(++pf_pdf_seed);

  return pdf;
}

void pf_pdf_uniform_free(pf_pdf_uniform_t* pdf)
{
  free(pdf);
}

pf_vector_t pf_pdf_uniform_sample(pf_pdf_uniform_t* pdf)
{
  double r;
  pf_vector_t pose;

  do
  {
    r = drand48();
    pose.v[0] = (pdf->max_x - pdf->min_x) * r + pdf->min_x;
    r = drand48();
    pose.v[1] = (pdf->max_y - pdf->min_y) * r + pdf->min_y;
    r = drand48();
    pose.v[2] = 2 * M_PI * ( r - 0.5);
  } 
  // while(contains(pdf, pose) == 0);
  while(0);
  return pose;
}

/**************************************************************************
 * Gaussian
 *************************************************************************/

// Create a gaussian pdf
pf_pdf_gaussian_t *pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
{
  pf_matrix_t cd;
  pf_pdf_gaussian_t *pdf;

  pdf = (pf_pdf_gaussian_t*)malloc(sizeof(pf_pdf_gaussian_t));

  pdf->x = x;
  pdf->cx = cx;
  //pdf->cxi = pf_matrix_inverse(cx, &pdf->cxdet);

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
  pdf->cd.v[0] = sqrt(cd.m[0][0]);
  pdf->cd.v[1] = sqrt(cd.m[1][1]);
  pdf->cd.v[2] = sqrt(cd.m[2][2]);

  // Initialize the random number generator
  //pdf->rng = gsl_rng_alloc(gsl_rng_taus);
  //gsl_rng_set(pdf->rng, ++pf_pdf_seed);
  srand48(++pf_pdf_seed);

  return pdf;
}


// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t *pdf)
{
  //gsl_rng_free(pdf->rng);
  free(pdf);
  return;
}


/*
// Compute the value of the pdf at some point [x].
double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t x)
{
  int i, j;
  pf_vector_t z;
  double zz, p;
  
  z = pf_vector_sub(x, pdf->x);

  zz = 0;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      zz += z.v[i] * pdf->cxi.m[i][j] * z.v[j];

  p =  1 / (2 * M_PI * pdf->cxdet) * exp(-zz / 2);
          
  return p;
}
*/


// Generate a sample from the the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t *pdf)
{
  int i, j;
  pf_vector_t r;
  pf_vector_t x;

  // Generate a random vector
  for (i = 0; i < 3; i++)
  {
    //r.v[i] = gsl_ran_gaussian(pdf->rng, pdf->cd.v[i]);
    r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);
  }

  for (i = 0; i < 3; i++)
  {
    x.v[i] = pdf->x.v[i];
    for (j = 0; j < 3; j++)
      x.v[i] += pdf->cr.m[i][j] * r.v[j];
  } 
  
  return x;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

#if 0

/**************************************************************************
 * Discrete
 * Note that GSL v1.3 and earlier contains a bug in the discrete
 * random generator.  A patched version of the the generator is included
 * in gsl_discrete.c.
 *************************************************************************/


// Create a discrete pdf
pf_pdf_discrete_t *pf_pdf_discrete_alloc(int count, double *probs)
{
  pf_pdf_discrete_t *pdf;

  pdf = calloc(1, sizeof(pf_pdf_discrete_t));

  pdf->prob_count = count;
  pdf->probs = malloc(count * sizeof(double));
  memcpy(pdf->probs, probs, count * sizeof(double));
  
  // Initialize the random number generator
  pdf->rng = gsl_rng_alloc(gsl_rng_taus);
  gsl_rng_set(pdf->rng, ++pf_pdf_seed);

  // Initialize the discrete distribution generator
  pdf->ran = gsl_ran_discrete_preproc(count, probs);

  return pdf;
}


// Destroy the pdf
void pf_pdf_discrete_free(pf_pdf_discrete_t *pdf)
{
  gsl_ran_discrete_free(pdf->ran);
  gsl_rng_free(pdf->rng);
  free(pdf->probs);  
  free(pdf);
  return;
}


// Compute the value of the probability of some element [i]
double pf_pdf_discrete_value(pf_pdf_discrete_t *pdf, int i)
{
  return pdf->probs[i];
}


// Generate a sample from the the pdf.
int pf_pdf_discrete_sample(pf_pdf_discrete_t *pdf)
{
  int i;
  
  i = gsl_ran_discrete(pdf->rng, pdf->ran);
  assert(i >= 0 && i < pdf->prob_count);

  return i;
}

#endif
