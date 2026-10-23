import math
import numpy as np
import pytest
from path_planning.astar import astar
from path_planning.rrt import rrt
from path_planning.collision import segment_free
from path_planning.maps import create_grid_map
from path_planning.metrics import path_length

def test_map_seed_and_endpoints():
    a=create_grid_map(seed=12);b=create_grid_map(seed=12)
    assert np.array_equal(a,b)
    assert a[5,10]==0 and a[95,95]==0

def test_astar_empty_optimal():
    path,expanded=astar(np.zeros((5,5),dtype=np.uint8),(0,0),(4,4))
    assert path[0]==(0,0) and path[-1]==(4,4)
    assert path_length(path)==pytest.approx(4*math.sqrt(2))
    assert expanded

def test_astar_no_path():
    grid=np.zeros((5,5),dtype=np.uint8);grid[2,:]=1
    path,_=astar(grid,(0,0),(4,4))
    assert path is None

def test_rrt_empty():
    path,nodes=rrt(np.zeros((20,20),dtype=np.uint8),(1,1),(18,18),seed=42,max_iter=3000)
    assert path is not None and path[0]==(1.5,1.5) and path[-1]==(18.5,18.5)
    assert all(segment_free(np.zeros((20,20),dtype=np.uint8),a,b) for a,b in zip(path,path[1:]))

def test_collision_corner():
    grid=np.zeros((3,3),dtype=np.uint8);grid[1,0]=1
    assert not segment_free(grid,(.5,.5),(1.5,1.5))

def test_collision_wall():
    grid=np.zeros((10,10),dtype=np.uint8);grid[5,5]=1
    assert not segment_free(grid,(2.5,2.5),(8.5,8.5))

def test_invalid_start():
    grid=np.zeros((5,5),dtype=np.uint8);grid[0,0]=1
    with pytest.raises(ValueError):astar(grid,(0,0),(4,4))
    with pytest.raises(ValueError):rrt(grid,(0,0),(4,4))

def test_rrt_same_start_goal():
    path,_=rrt(np.zeros((5,5),dtype=np.uint8),(1,1),(1,1))
    assert path==[(1.5,1.5)]
