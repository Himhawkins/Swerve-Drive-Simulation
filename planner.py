import pygame
import numpy as np
import scipy.ndimage
from skimage.graph import route_through_array
import robot_config as config

class Planner:
    def __init__(self, walls, width, height):
        self.width = width
        self.height = height
        self.cost_map = self.create_potential_field(walls)
        self.path = []
        
        # Surface for visualization (Heatmap)
        self.vis_surface = self.create_heatmap_surface()

    def create_potential_field(self, walls):
        """
        Creates a gradient cost map.
        Pixels near walls have HIGH cost. Pixels far from walls have LOW cost.
        """
        # 1. Create a binary mask (0 = Wall, 1 = Space)
        temp_surf = pygame.Surface((self.width, self.height))
        temp_surf.fill((1, 1, 1)) # Fill with Space
        for wall in walls:
            pygame.draw.rect(temp_surf, (0, 0, 0), wall) # Draw walls as 0
        
        # Convert to numpy array (Transposed for x,y handling)
        # We take the red channel [:,:,0] since it's grayscale
        mask = pygame.surfarray.array3d(temp_surf)[:,:,0]
        
        # 2. Euclidean Distance Transform
        # Calculates distance from every pixel to the nearest 0 (wall)
        distance_map = scipy.ndimage.distance_transform_edt(mask)
        
        # 3. Invert Distance to create Cost
        # Formula: Cost = Base_Cost + (Factor / (Distance + epsilon))
        # +1 prevents division by zero. 
        # The '200' factor controls how "repulsive" the walls are.
        cost_map = 1 + (500 / (distance_map + 1))
            
        robot_radius = config.PLAYER_SIZE / 2 + 1
        # 4. Hard block walls (Make actual wall pixels effectively infinite cost)
        # Wherever the distance is 0 (the wall itself), set cost extremely high
        cost_map[distance_map < robot_radius] =  1e6 #np.inf
        # cost_map[distance_map == 0] = 100000
        
        return cost_map

    def create_heatmap_surface(self):
        """
        Creates a visual overlay to see the potential field.
        """
        # Normalize cost map to 0-255 for display
        # We clip high values so the gradient is visible
        display_map = self.cost_map.copy()
        display_map = np.clip(display_map, 0, 50) # Clip costs above 50 for better contrast
        display_map = (display_map / 50) * 255    # Scale to 0-255
        
        # Create surface
        surf = pygame.surfarray.make_surface(np.stack((display_map, display_map/2, display_map/4), axis=-1))
        surf.set_alpha(150) # Make it transparent
        return surf

    def get_path(self, start_pos, end_pos):
        start_node = (int(start_pos[0]), int(start_pos[1]))
        end_node = (int(end_pos[0]), int(end_pos[1]))

        try:
            indices, weight = route_through_array(
                self.cost_map, 
                start_node, 
                end_node, 
                geometric=True,
                fully_connected=True 
            )
            path = np.array([p[::1] for p in indices])
            return path
        except:
            return []