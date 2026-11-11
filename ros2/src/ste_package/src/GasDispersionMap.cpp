#include <math.h>
// #include "nvwa/debug_new.h"
#include <fstream>
#include "ste_package/GasDispersionMap.h"

extern SourceBounds source_bounds;
GasDispersionMap::GasDispersionMap(int index_sx, int index_sy)
{
	idx_sx = index_sx;
	idx_sy = index_sy;
	indices_to_position_2D(idx_sx, idx_sy, &sx, &sy);

	gasConcentrationMap.assign(source_bounds.x_nbins, vector<double>(source_bounds.y_nbins, 0.0));
}

void GasDispersionMap::log_map(const char *filename, char writing_mode)
{

	FILE *gas_dispersion_maps = fopen(filename, &writing_mode);
	fprintf(gas_dispersion_maps, "%.9f, %9f", sx, sy);
	for (int i = 0; i < source_bounds.x_nbins; i++)
	{
		for (int j = 0; j < source_bounds.y_nbins; j++)
		{
			fprintf(gas_dispersion_maps, "%.9f, ", gasConcentrationMap[i][j]);
		}
	}
	fprintf(gas_dispersion_maps, "\n");
	fclose(gas_dispersion_maps);
}

double GasDispersionMap::get_concentration(int idx_x, int idx_y)
{
	if (idx_x < 0 || idx_x >= source_bounds.x_nbins || idx_y < 0 || idx_y >= source_bounds.y_nbins)
	{
		printf("Error: index out of range\n");
		return 0;
	}
	return gasConcentrationMap[idx_x][idx_y];
}