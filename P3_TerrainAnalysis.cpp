#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>
#include <vector>
#include <tuple>

#define SQRT_TWO 1.4142136f
enum NeighborType {
	BR,
	BL,
	TR,
	TL
};

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
	return false;
}

bool check_block(GridPos pos, NeighborType type)
{
	if (type == BR)
	{
		GridPos B = pos;
		GridPos R = pos;
		--B.col;
		--R.row;

		if (terrain->is_wall(B) && terrain->is_wall(R))
			return true;
		else
			return false;
	}

	if (type == BL)
	{
		GridPos B = pos;
		GridPos L = pos;
		++B.col;
		--L.row;

		if (terrain->is_wall(B) && terrain->is_wall(L))
			return true;
		else
			return false;
	}

	if (type == TR)
	{
		GridPos T = pos;
		GridPos R = pos;
		--T.col;
		++R.row;

		if (terrain->is_wall(T) && terrain->is_wall(R))
			return true;
		else
			return false;
	}

	if (type == TL)
	{
		GridPos T = pos;
		GridPos L = pos;
		++T.col;
		++L.row;

		if (terrain->is_wall(T) && terrain->is_wall(L))
			return true;
		else
			return false;
	}
	

	return false;
}

bool check_next_to_visible(GridPos pos, NeighborType type)
{
	if (type == BR)
	{
		GridPos B = pos;
		GridPos R = pos;
		--B.col;
		--R.row;

		if (terrain->is_wall(B) || terrain->is_wall(R))
			return true;
		else
			return false;
	}

	if (type == BL)
	{
		GridPos B = pos;
		GridPos L = pos;
		++B.col;
		--L.row;

		if (terrain->is_wall(B) || terrain->is_wall(L))
			return true;
		else
			return false;
	}

	if (type == TR)
	{
		GridPos T = pos;
		GridPos R = pos;
		--T.col;
		++R.row;

		if (terrain->is_wall(T) || terrain->is_wall(R))
			return true;
		else
			return false;
	}

	if (type == TL)
	{
		GridPos T = pos;
		GridPos L = pos;
		++T.col;
		++L.row;

		if (terrain->is_wall(T) || terrain->is_wall(L))
			return true;
		else
			return false;
	}


	return false;
}


float distance_to_closest_wall(int row, int col)
{
	/*
		Check the euclidean distance from the given cell to every other wall cell,
		as well as the distance to the edges (they count as walls for this),
		and return the smallest distance.  Make use of the is_wall member function
		in the global terrain to determine if a cell is a wall or not.
	*/

	float distance = 100.f;

	int row_index[2];
	int col_index[2];

	GridPos curr;
	curr.row = row;
	curr.col = col;

	if (terrain->is_wall(curr))
		return 0;

	int offset = 1;
	while (1)
	{
		row_index[0] = row - offset;
		row_index[1] = row + offset;
		col_index[0] = col - offset;
		col_index[1] = col + offset;

		for (int i = 0; i < 2; ++i)
		{
			for (int j = col_index[0]; j <= col_index[1]; ++j)
			{
				
				curr.row = row_index[i];
				curr.col = j;

				if (!terrain->is_valid_grid_position(curr) || terrain->is_wall(curr))
				{
					float xDiff = (float)std::abs(curr.col - col);
					float yDiff = (float)std::abs(curr.row - row);

					xDiff *= xDiff;
					yDiff *= yDiff;
					float cost = std::sqrtf(xDiff + yDiff);

					if (cost < distance)
						distance = cost;
				}
			}
		}

		for (int i = 0; i < 2; ++i)
		{
			for (int j = row_index[0] + 1; j <= row_index[1] - 1; ++j)
			{
				curr.row = j;
				curr.col = col_index[i];

				if (!terrain->is_valid_grid_position(curr) || terrain->is_wall(curr))
				{
					float xDiff = (float)std::abs(curr.col - col);
					float yDiff = (float)std::abs(curr.row - row);

					xDiff *= xDiff;
					yDiff *= yDiff;
					float cost = std::sqrtf(xDiff + yDiff);

					if (cost < distance)
						distance = cost;
				}
			}
		}

		if (distance != 100)
			break;

		++offset;
	}

	return distance;
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
	/*
		Two cells (row0, col0) and (row1, col1) are visible to each other if a line
		between their centerpoints doesn't intersect the four boundary lines of every
		wall cell.  You should puff out the four boundary lines by a very tiny amount
		so that a diagonal line passing by the corner will intersect it.  Make use of the
		is_wall member function in the global terrain to determine if a cell is a wall or not.
	*/


	float offset = 0.51f;

	float slope_x = (float)(row1 - row0) / (float)(col1 - col0);
	float constant_x = row0 - slope_x * col0;
	float slope_y = 1.0f / slope_x;
	float constant_y = col0 - slope_y*row0;

	int i_min = row0;
	int i_max = row1;
	int j_min = col0;
	int j_max = col1;

	if (row0 > row1)
	{
		i_min = row1;
		i_max = row0;
	}
	if (col0 > col1)
	{
		j_min = col1;
		j_max = col0;
	}

	for (int i = i_min; i <= i_max; ++i)
	{
		for (int j = j_min; j <= j_max; ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;
			if (!terrain->is_valid_grid_position(curr))
				continue;

			if (!terrain->is_wall(curr))
				continue;

			float yMin = slope_x * (float)(j - offset) + constant_x;
			float yMax = slope_x * (float)(j + offset) + constant_x;
			if (yMin >= (float)(i - offset) && yMin <= (float)(i + offset))
				return false;
			if (yMax >= (float)(i - offset) && yMax <= (float)(i + offset))
				return false;

			float xMin = slope_y * (float)(i - offset) + constant_y;
			float xMax = slope_y * (float)(i + offset) + constant_y;
			if (xMin >= (float)(j - offset) && xMin <= (float)(j + offset))
				return false;
			if (xMax >= (float)(j - offset) && xMax <= (float)(j + offset))
				return false;

		}
	}


	return true; // REPLACE THIS
}

void analyze_openness(MapLayer<float> &layer)
{
	/*
		Mark every cell in the given layer with the value 1 / (d * d),
		where d is the distance to the closest wall or edge.  Make use of the
		distance_to_closest_wall helper function.
	*/

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			float d = distance_to_closest_wall(i, j);
			layer.set_value(i, j, 1.0f / (d*d));
		}
	}

}

void analyze_visibility(MapLayer<float> &layer)
{
	/*
		Mark every cell in the given layer with the number of cells that
		are visible to it, divided by 160 (a magic number that looks good).  Make sure
		to cap the value at 1.0 as well.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;

			if (terrain->is_wall(curr))
				continue;

			int visible_num = 0;
			for (int k_r = 0; k_r < terrain->get_map_height(); ++k_r)
			{
				for (int k_c = 0; k_c < terrain->get_map_width(); ++k_c)
				{
					if (i == k_r && j == k_c)
						continue;

					if (is_clear_path(i, j, k_r, k_c))
						++visible_num;
				}
			}

			layer.set_value(curr, visible_num / 160.f);
		}
	}


}

void analyze_visble_to_cell(MapLayer<float> &layer, int row, int col)
{
	/*
		For every cell in the given layer mark it with 1.0
		if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
		or 0.0 otherwise.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;

			if (terrain->is_wall(curr))
				continue;

			if (is_clear_path(row, col, i, j))
				layer.set_value(i, j, 1.0f);

			else
				layer.set_value(curr, 0.0f);
			

		}
	}



	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			if (layer.get_value(i, j) != 1.0f)
				continue;

			for (int k_r = i - 1; k_r <= i + 1; ++k_r)
			{
				for (int k_c = j - 1; k_c <= j + 1; ++k_c)
				{
					GridPos neighbor;
					neighbor.col = k_c;
					neighbor.row = k_r;

					if (!terrain->is_valid_grid_position(neighbor) || terrain->is_wall(neighbor))
						continue;

					int offset = std::abs(k_r - i) + std::abs(k_c - j);
					if (offset != 1.0f)
					{
						bool blocked = false;
						bool not_walkable = false;
						//BR
						if (k_r > i && k_c > j)
						{
							blocked = check_block(neighbor, NeighborType::BR);
							not_walkable = check_next_to_visible(neighbor, NeighborType::BR);
						}
						//BL
						if (k_r > i && k_c < j)
						{
							blocked = check_block(neighbor, NeighborType::BL);
							not_walkable = check_next_to_visible(neighbor, NeighborType::BL);
						}
						//TR
						if (k_r < i && k_c > j)
						{
							blocked = check_block(neighbor, NeighborType::TR);
							not_walkable = check_next_to_visible(neighbor, NeighborType::TR);
						}
						//TL
						if (k_r < i && k_c < j)
						{
							blocked = check_block(neighbor, NeighborType::TL);
							not_walkable = check_next_to_visible(neighbor, NeighborType::TL);
						}

						if (blocked || not_walkable)
							continue;


						
					}
					
					if (layer.get_value(neighbor) == 0.0f)
						layer.set_value(neighbor, 0.5f);
				}
			}


		}
	}



}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
	/*
		For every cell in the given layer that is visible to the given agent,
		mark it as 1.0, otherwise don't change the cell's current value.

		You must consider the direction the agent is facing.  All of the agent data is
		in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

		Give the agent a field of view slighter larger than 180 degrees.

		Two cells are visible to each other if a line between their centerpoints doesn't
		intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
		helper function.
	*/

	GridPos agent_position = terrain->get_grid_position(agent->get_position());
	Vec2 agent_view(agent->get_forward_vector().z, agent->get_forward_vector().x);

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			
			Vec2 cell((float)(j-agent_position.col), (float)(i-agent_position.row));
			cell.Normalize();

			if (agent_view.Dot(cell) >= -0.1f && is_clear_path(agent_position.row, agent_position.col, i, j))
				layer.set_value(i,j,1.0f);
			
		}
	}


}


void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
	/*
		For every cell in the given layer:

			1) Get the value of each neighbor and apply decay factor
			2) Keep the highest value from step 1
			3) Linearly interpolate from the cell's current value to the value from step 2
			   with the growing factor as a coefficient.  Make use of the lerp helper function.
			4) Store the value from step 3 in a temporary layer.
			   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/
	float temp[40][40];

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;

			if (terrain->is_wall(curr))
			{
				temp[i][j] = 0.0f;
				continue;
			}
			
			float max = 0;
			//neighbor
			for (int n_row = i - 1; n_row <= i + 1; ++n_row)
			{
				for (int n_col = j - 1; n_col <= j + 1; ++n_col)
				{
					if (n_row == i && n_col == j)
						continue;

					GridPos neighbor;
					neighbor.col = n_col;
					neighbor.row = n_row;

					if (!terrain->is_valid_grid_position(neighbor))
						continue;

					if (terrain->is_wall(neighbor))
						continue;
					

					int offset = std::abs(n_row - i) + std::abs(n_col - j);
					float d = 1.0f;
					if (offset != 1.0f)
					{
						bool blocked = false;
						//BR
						if (n_row > i && n_col > j)
							blocked = check_block(neighbor, NeighborType::BR);
						//BL
						if (n_row > i && n_col < j)
							blocked = check_block(neighbor, NeighborType::BL);
						//TR
						if (n_row < i && n_col > j)
							blocked = check_block(neighbor, NeighborType::TR);
						//TL
						if (n_row < i && n_col < j)
							blocked = check_block(neighbor, NeighborType::TL);

						if (blocked)
							continue;

						d = SQRT_TWO;
					}

					float influence = layer.get_value(n_row, n_col)*exp(-d * decay);
					if (influence > max)
						max = influence;
				}
			}
			
			temp[i][j] = lerp(layer.get_value(i, j), max, growth);
		}
	}

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			layer.set_value(i, j, temp[i][j]);
		}
	}


}



void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
	/*
		Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

		For every cell in the given layer:

		1) Get the value of each neighbor and apply decay factor
		2) Keep the highest ABSOLUTE value from step 1
		3) Linearly interpolate from the cell's current value to the value from step 2
		   with the growing factor as a coefficient.  Make use of the lerp helper function.
		4) Store the value from step 3 in a temporary layer.
		   A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

		After every cell has been processed into the temporary layer, write the temporary layer into
		the given layer;
	*/

	float temp[40][40];
	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;

			if (terrain->is_wall(curr))
			{
				temp[i][j] = 0.0f;
				continue;
			}

			float abs_max = 0;
			float max = 0;
			//neighbor
			for (int n_row = i - 1; n_row <= i + 1; ++n_row)
			{
				for (int n_col = j - 1; n_col <= j + 1; ++n_col)
				{
					if (n_row == i && n_col == j)
						continue;

					GridPos neighbor;
					neighbor.col = n_col;
					neighbor.row = n_row;

					if (!terrain->is_valid_grid_position(neighbor))
						continue;

					if (terrain->is_wall(neighbor))
						continue;
					

					int offset = std::abs(n_row - i) + std::abs(n_col - j);
					float d = 1.0f;
					if (offset != 1.0f)
					{
						bool blocked = false;
						//BR
						if (n_row > i && n_col > j)
							blocked = check_block(neighbor, NeighborType::BR);
						//BL
						if (n_row > i && n_col < j)
							blocked = check_block(neighbor, NeighborType::BL);
						//TR
						if (n_row < i && n_col > j)
							blocked = check_block(neighbor, NeighborType::TR);
						//TL
						if (n_row < i && n_col < j)
							blocked = check_block(neighbor, NeighborType::TL);

						if (blocked)
							continue;

						d = SQRT_TWO;
					}

					float influence = layer.get_value(n_row, n_col)*exp(-d * decay);
					if (std::abs(influence) > abs_max)
					{
						abs_max = std::abs(influence);
						max = influence;
					}
				}
			}
			temp[i][j] = lerp(layer.get_value(i, j), max, growth);
		}
	}

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			layer.set_value(i, j, temp[i][j]);
		}
	}


}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
	/*
		Determine the maximum value in the given layer, and then divide the value
		for every cell in the layer by that amount.  This will keep the values in the
		range of [0, 1].
	*/

	float high = 0.f;
	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			float layer_val = layer.get_value(i, j);
			if (layer_val > high)
				high = layer_val;
		}
	}

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			
			float layer_val = layer.get_value(i, j);
			if (layer_val <= 0)
				continue;

			layer.set_value(i, j, layer_val / high);
				
		}
	}

}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
	/*
		Similar to the solo version, but you need to track greatest positive value AND
		the least (furthest from 0) negative value.

		For every cell in the given layer, if the value is currently positive divide it by the
		greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
		(so that it remains a negative number).  This will keep the values in the range of [-1, 1].
	*/

	float high_pos = 0.f;
	float high_neg = 0.f;
	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			float layer_val = layer.get_value(i, j);
			if (layer_val >= 0 && layer_val > high_pos)
				high_pos = layer_val;
			if (layer_val < 0 && layer_val < high_neg)
				high_neg = layer_val;
		}
	}

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			float layer_val = layer.get_value(i, j);
			if (layer_val < 0)
				layer.set_value(i, j, -layer_val / high_neg);
			if(layer_val > 0)
				layer.set_value(i, j, layer_val / high_pos);

		}
	}


}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
	/*
		First, clear out the old values in the map layer by setting any negative value to 0.
		Then, for every cell in the layer that is within the field of view cone, from the
		enemy agent, mark it with the occupancy value.

		If the tile is close enough to the enemy (less than closeDistance),
		you only check if it's visible to enemy.  Make use of the is_clear_path
		helper function.  Otherwise, you must consider the direction the enemy is facing too.
		This creates a radius around the enemy that the player can be detected within, as well
		as a fov cone.
	*/

	auto op=[](float& value)
	{
		if (value < 0.f)
			value = 0.0f;
	};

	layer.for_each(op);

	GridPos enemyPos = terrain->get_grid_position(enemy->get_position());

	Vec2 f_look(enemy->get_forward_vector().z, enemy->get_forward_vector().x);
	float closeDistance_2 = closeDistance * closeDistance;

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			//close distance
			float xDiff = (float)(j - enemyPos.col);
			float yDiff = (float)(i - enemyPos.row);
			if (xDiff *xDiff  + yDiff*yDiff <= closeDistance_2)
			{
				if (is_clear_path(enemyPos.row, enemyPos.col, i, j))
					layer.set_value(i, j, occupancyValue);
			}
			else
			{
				Vec2 cell((float)(j - enemyPos.col), (float)(i - enemyPos.row));
				cell.Normalize();
				
				float angle = (fovAngle/2.f)*PI/180;
				float dot = f_look.Dot(cell);
				if (f_look.Dot(cell) >= cos(angle))
				{
					if(is_clear_path(enemyPos.row, enemyPos.col, i, j))
						layer.set_value(i, j, occupancyValue);
				}

			}


		}
	}




}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
	/*
		Check if the player's current tile has a negative value, ie in the fov cone
		or within a detection radius.
	*/

	const auto &playerWorldPos = player->get_position();

	const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

	// verify a valid position was returned
	if (terrain->is_valid_grid_position(playerGridPos) == true)
	{
		if (layer.get_value(playerGridPos) < 0.0f)
		{
			return true;
		}
	}

	// player isn't in the detection radius or fov cone, OR somehow off the map
	return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
	/*
		Attempt to find a cell with a value of 1.0, and then set it as the new target,
		using enemy->path_to.

		If there are multiple cells with 1.0, then pick the cell closest to the enemy.

		Return whether a target cell was found.
	*/

	float dist = 3200.f;
	GridPos target;
	target.col = -1;
	target.row = -1;
	GridPos enemyPos = terrain->get_grid_position(enemy->get_position());

	for (int i = 0; i < terrain->get_map_height(); ++i)
	{
		for (int j = 0; j < terrain->get_map_width(); ++j)
		{
			GridPos curr;
			curr.row = i;
			curr.col = j;

			if (terrain->is_wall(curr))
				continue;

			if (layer.get_value(i, j) == 1.0f)
			{
				float xDiff = (float)(j - enemyPos.col);
				float yDiff = (float)(i - enemyPos.row);
				float d = xDiff * xDiff + yDiff * yDiff;
				if ( d < dist)
				{
					dist = d;
					target.row = i;
					target.col = j;
				}
			}
		}
	}

	if (target.row == -1 && target.col == -1)
		return false;
	else
	{
		enemy->path_to(terrain->get_world_position(target));
		return true;
	}

}

void player_flee(MapLayer<float>& layer, AStarAgent * player, AStarAgent* enemy)
{
	GridPos player_pos = terrain->get_grid_position(player->get_position());
	GridPos enemy_pos = terrain->get_grid_position(enemy->get_position());

	Vec2 f_look(enemy->get_forward_vector().z, enemy->get_forward_vector().x);


	float distance = 100.f;

	int row_index[2];
	int col_index[2];

	GridPos curr = player_pos;
	GridPos goal;

	if (terrain->is_wall(player_pos))
		return;

	int offset = 1;
	while (1)
	{
		row_index[0] = curr.row - offset;
		row_index[1] = curr.row + offset;
		col_index[0] = curr.col - offset;
		col_index[1] = curr.col + offset;

		for (int i = 0; i < 2; ++i)
		{
			for (int j = col_index[0]; j <= col_index[1]; ++j)
			{

				curr.row = row_index[i];
				curr.col = j;

				if (!terrain->is_valid_grid_position(curr))
					continue;

				Vec2 cell((float)(j - enemy_pos.col), (float)(i - enemy_pos.row));
				cell.Normalize();

				if (f_look.Dot(cell) < 0)
					continue;

				if (layer.get_value(curr) >= 0.0f)
				{

					goal = curr;

					player->path_to(terrain->get_world_position(goal));
					return;
				}
			}
		}

		for (int i = 0; i < 2; ++i)
		{
			for (int j = row_index[0] + 1; j <= row_index[1] - 1; ++j)
			{
				curr.row = j;
				curr.col = col_index[i];

				if (!terrain->is_valid_grid_position(curr))
					continue;

				Vec2 cell((float)(j - enemy_pos.col), (float)(i - enemy_pos.row));
				cell.Normalize();

				if (f_look.Dot(cell) < 0)
					continue;

				if (layer.get_value(curr) >= 0.0f)
				{

					goal = curr;

					player->path_to(terrain->get_world_position(goal));
					return;
				}
			}
		}

		if (distance != 100)
		{
			player->path_to(terrain->get_world_position(goal));
			return;
		}

		++offset;
	}


}

bool player_find_hiding_place(MapLayer<float>& layer, AStarAgent* player)
{

	GridPos player_pos = terrain->get_grid_position(player->get_position());

	float distance = 100.f;

	int row_index[2];
	int col_index[2];

	GridPos curr = player_pos;
	GridPos goal;

	if (terrain->is_wall(player_pos))
		return false;

	int offset = 1;
	while (1)
	{
		row_index[0] = curr.row - offset;
		row_index[1] = curr.row + offset;
		col_index[0] = curr.col - offset;
		col_index[1] = curr.col + offset;

		for (int i = 0; i < 2; ++i)
		{
			for (int j = col_index[0]; j <= col_index[1]; ++j)
			{

				curr.row = row_index[i];
				curr.col = j;

				if (!terrain->is_valid_grid_position(curr))
					continue;


				if (layer.get_value(curr) < 0.5f)
				{

					goal = curr;


					player->path_to(terrain->get_world_position(goal));
					return true;
				}
			}
		}

		for (int i = 0; i < 2; ++i)
		{
			for (int j = row_index[0] + 1; j <= row_index[1] - 1; ++j)
			{
				curr.row = j;
				curr.col = col_index[i];

				if (!terrain->is_valid_grid_position(curr))
					continue;

				if (layer.get_value(curr) < 0.5f)
				{
					goal = curr;

					player->path_to(terrain->get_world_position(goal));
					return true;
				}
			}
		}

		++offset;
	}



	return false;
}


