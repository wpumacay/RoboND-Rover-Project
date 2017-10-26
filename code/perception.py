import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def rgb_thresh( img, lo, hi ) :
    _res = cv2.inRange( img, lo, hi )
    return _res

def hsv_thresh( img, lo, hi ) :
    _img_hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )
    _res = cv2.inRange( _img_hsv, lo, hi )
    return _res

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    yaw = yaw * np.pi / 180.
    # Apply a rotation
    xpix_rotated = np.cos( yaw ) * xpix - np.sin( yaw ) * ypix
    ypix_rotated = np.sin( yaw ) * xpix + np.cos( yaw ) * ypix
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = ( xpix_rot / scale ) + xpos
    ypix_translated = ( ypix_rot / scale ) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 

    # NOTE: camera image is coming to you in Rover.img

    # 1) Define source and destination points for perspective transform

    _dst_size = 5 
    _btm_offset = 6
    _w = Rover.img.shape[1]
    _h = Rover.img.shape[0]
    _src_pts = np.float32( [ [14, 140], [301 ,140],[200, 96], [118, 96] ] )
    _dst_pts = np.float32( [ [ _w / 2 - _dst_size, _h - _btm_offset ],
                             [ _w / 2 + _dst_size, _h - _btm_offset ],
                             [ _w / 2 + _dst_size, _h - 2 * _dst_size - _btm_offset ], 
                             [ _w / 2 - _dst_size, _h - 2 * _dst_size - _btm_offset ],
                           ])

    # 2) Apply perspective transform

    _img_warped = perspect_transform( Rover.img, _src_pts, _dst_pts )

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    # ranges for terrain segmentation
    _hsv_threshold_terrain_lo = np.array( [ 0, 28, 184 ] )
    _hsv_threshold_terrain_hi = np.array( [ 138, 60, 255 ] )
    # ranges for obstacle segmentation
    _hsv_threshold_obstacles_lo = np.array( [ 0, 0, 0 ] )
    _hsv_threshold_obstacles_hi = np.array( [ 220, 255, 181 ] )
    # ranges for rock segmentation
    _hsv_threshold_rocks_lo = np.array( [ 20, 98, 40 ] )
    _hsv_threshold_rocks_hi = np.array( [ 100, 255, 255 ] )
    
    #_img_threshed_terrain   = hsv_thresh( _img_warped, _hsv_threshold_terrain_lo, _hsv_threshold_terrain_hi )
    _img_threshed_terrain   = rgb_thresh( _img_warped, ( 160, 160, 160 ), ( 255, 255, 255 ) ) 
    #_img_threshed_obstacles = hsv_thresh( _img_warped, _hsv_threshold_obstacles_lo, _hsv_threshold_obstacles_hi )
    _img_threshed_obstacles = rgb_thresh( _img_warped, ( 0, 0, 0 ), ( 160, 160, 160 ) ) 
    _img_threshed_rocks     = hsv_thresh( _img_warped, _hsv_threshold_rocks_lo, _hsv_threshold_rocks_hi )

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:, :, 0] = _img_threshed_obstacles
    Rover.vision_image[:, :, 1] = _img_threshed_rocks
    Rover.vision_image[:, :, 2] = _img_threshed_terrain

    # 5) Convert map image pixel values to rover-centric coords

    x_navigable_rover, y_navigable_rover = rover_coords( _img_threshed_terrain )
    x_obstacles_rover, y_obstacles_rover = rover_coords( _img_threshed_obstacles )
    x_rocks_rover, y_rocks_rover         = rover_coords( _img_threshed_rocks )

    # 6) Convert rover-centric pixel values to world coordinates

    _rover2world_scale = 10. # 1pix in world is 1m, whereas 1pix in rover is 0.1m
    _world_size = 200
    
    _x = Rover.pos[0]
    _y = Rover.pos[1]
    _yaw = Rover.yaw
    
    x_navigable_world, y_navigable_world = pix_to_world( x_navigable_rover, y_navigable_rover,
                                                         _x, _y, _yaw, _world_size, _rover2world_scale )
    x_obstacles_world, y_obstacles_world = pix_to_world( x_obstacles_rover, y_obstacles_rover,
                                                         _x, _y, _yaw, _world_size, _rover2world_scale )
    x_rocks_world, y_rocks_world         = pix_to_world( x_rocks_rover, y_rocks_rover,
                                                         _x, _y, _yaw, _world_size, _rover2world_scale )

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    Rover.worldmap[y_obstacles_world, x_obstacles_world, 0] += 1
    Rover.worldmap[y_rocks_world, x_rocks_world, 1] += 1
    Rover.worldmap[y_navigable_world, x_navigable_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    Rover.nav_dists, Rover.nav_angles = to_polar_coords( x_navigable_rover, y_navigable_rover )
    
    return Rover