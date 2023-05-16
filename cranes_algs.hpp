///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes 
{
// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) 
{
  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);

  // Compute maximum path length, and check that it is legal.
  const size_t max_steps = setting.rows() + setting.columns() - 2;
  assert(max_steps < 64);

  // TODO: implement the exhaustive search algorithm, then delete this
  // comment.
  path best(setting);

  for(size_t steps = 0; steps <= max_steps; steps++) 
  {
    std::vector<step_direction> directions(steps, STEP_DIRECTION_EAST);
    directions.resize(max_steps, STEP_DIRECTION_SOUTH);

    do 
    {
      path current(setting);

      for(const step_direction& direction : directions) 
      {
        if(current.is_step_valid(direction)) 
        {
          current.add_step(direction);
        } 
        else 
        {
          break;
        }
      }

      if(current.total_cranes() > best.total_cranes()) 
      {
        best = current;
      }
    } while(std::next_permutation(directions.begin(), directions.end()));
  }

  return best;
}

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
//path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) 
{

  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);
  
  using cell_type = std::optional<path>;

  std::vector<std::vector<cell_type> > A(setting.rows(),
                                        std::vector<cell_type>(setting.columns()));

  A[0][0] = path(setting);
  assert(A[0][0].has_value());

  unsigned int most_cranes = 0;
  coordinate best_row_path = 0;
  coordinate best_column_path = 0;

  for(coordinate r = 0; r < setting.rows(); ++r)
  {
    for(coordinate c = 0; c < setting.columns(); ++c) 
    {
      if(setting.get(r, c) == CELL_BUILDING)
      {
        A[r][c].reset();
        continue;
      }

      cell_type from_above = std::nullopt;
      cell_type from_left = std::nullopt;

      if(r != 0 && setting.get(r-1, c) != CELL_BUILDING)
      {
        if(A[r-1][c].has_value())
        {
          from_above.emplace(A[r-1][c].value());
          from_above->add_step(STEP_DIRECTION_SOUTH);
        }
      }
    
      if(c != 0 && setting.get(r, c-1) != CELL_BUILDING) 
      {
        if(A[r][c-1].has_value()) 
        {
          from_left.emplace(A[r][c-1].value());
          from_left->add_step(STEP_DIRECTION_EAST);
        }
      }
    
      if(from_above.has_value() && from_left.has_value()) 
      {
        A[r][c] = from_above->total_cranes() >= from_left->total_cranes() ? from_above : from_left;
      } 
      else if(from_above.has_value() && !from_left.has_value()) 
      {
        A[r][c] = from_above;
      } 
      else if(!from_above.has_value() && from_left.has_value()) 
      {
        A[r][c] = from_left;
      }

      if(A[r][c].has_value() && A[r][c]->total_cranes() > most_cranes) 
      {
        most_cranes = A[r][c]->total_cranes();
        best_row_path = r;
        best_column_path = c;
      }
    }
  }    

  cell_type *best = &A[best_row_path][best_column_path]; 
  assert(best->has_value());
  
  return **best;
}

}


