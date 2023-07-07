#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

from point_cloud_processing import *
from utils import *
from constants import *


class ObjectManager: 
    """
    A class for managing objects. The managed objects need to implement a distance, update, quality and tobytes method. 

    Attributes:
        _min_dist (float): The minimum distance threshold for considering an object as new. 
        _objects (list): A list of objects.

    Methods:
        __init__(self, min_dist): Initializes the ObjectManager instance.
        add(self, object): Adds an object to the object manager.
        get_all(self): Returns all objects in the object manager.
        get_all_bytes(self): Returns all objects serialized as a byte string.
        _find_closest(self, object): Finds the closest object in the object manager to the given object.
    """

    def __init__(self, min_dist, log) -> None:
        self._min_dist = min_dist
        self._objects = []
        self.log = log

    def add(self, object): 
        """
        Adds an object to the object manager.
        If a similar object is found within the minimum distance threshold, it updates the closest object.
        Otherwise, it adds the new object to the object manager.

        Args:
            object: The object to be added.
        
        Returns:
            None
        """
        if object.quality < MIN_QUALITY:
            self.log("Detection has low quality")
            return

        closest, min_dist = self._find_closest(object)

        self.log("min. distance to closest similar object={}".format(min_dist))
        # update if the closest object is closer than the min_dist threshold
        if min_dist < self._min_dist: 
            self.log("Updating closest object")
            closest.update(object)
        else: 
            self.log("Adding new object")
            self._objects.append(object)

    def get_all(self): 
        return self._objects
    
    def get_all_bytes(self):
        """
        Returns all objects serialized as one byte string.

        Returns:
            bytes: Objects represented in bytes format.
        """
        b = b""
        for o in self._objects: 
            b += o.tobytes()
        return b 
     
    def _find_closest(self, object): 
        """
        Finds the closest object in the object manager to the given object.

        Args:
            object: The object for which to find the closest object.

        Returns:
            tuple: A tuple containing the closest object and the distance to it.
        """
        min_dist = INF
        closest = None
        for c in self._objects: 
            dist = object.distance(c)
            if dist < min_dist: 
                closest = c 
                min_dist = dist 
        return closest, min_dist


class Plane: 
    """
    A class representing a plane.

    Attributes:
        _corners (ndarray): The corners of the plane.
        _class_id (int): The class ID of the plane.
        _fit_rate (float): The fit rate of the plane.
        quality (float): Messure in [0,1] of how "good" the detection is 

    Methods:
        __init__(self, corners, class_id, fit_rate): Initializes the Plane instance.
        distance(self, other_plane): Calculates the distance between the plane and another plane.
        update(self, p): Updates the plane with another plane's information.
        area(self): Calculates the area of the plane.
        get_corners(self): Returns the corners of the plane.
        get_class_id(self): Returns the class ID of the plane.
        tobytes(self): Converts the plane object to bytes.
    """


    def __init__(self, corners, class_id, fit_rate): 
        """
        Initializes the Plane instance.

        Args:
            corners (ndarray): The corners of the plane.
            class_id (int): The class ID of the plane.
            fit_rate (float): The fit rate of the plane.
        
        Returns:
            None
        """
        self._corners = corners
        self._class_id = class_id    
        self._fit_rate = fit_rate
        self.quality = min(1, fit_rate/FIT_RATE_NORMALIZATION)
    
    
    def distance(self, other_plane):
        """
        Calculates the distance between the plane and another plane.

        If the class IDs of the planes don't match, returns infinity (INF).
        Otherwise, calculates the distance based on the area factor and squared difference of corners.

        Args:
            other_plane (Plane): The other plane to calculate the distance from.

        Returns:
            float: The distance between the planes.
        """
        if self._class_id != other_plane.get_class_id():
            return INF 
        area_factor = min(self.area() / AREA_NORMALIZATION, 1)
        sum_squared_diff = np.sum((self._corners - other_plane.get_corners()) ** 2)
        return sum_squared_diff / area_factor

    def update(self, p): 
        """
        Updates the plane with another plane's information.

        Calculates the update weight based on the area factor and fit rate of the plane.
        Updates the corners of the plane using the weighted average.

        Args:
            p (Plane): The plane to update from.

        Retruns:
            None
        """
        area_factor = min(self.area() / AREA_NORMALIZATION, 1)
        # k = PLANE_UPDATE_WEIGHT * area_factor * self._fit_rate ** 2
        k = PLANE_UPDATE_WEIGHT
        log("Plane update weight={}".format(k))
        self._corners = self._corners * (1-k) + p.get_corners() * k

    def area(self):
        """
        Calculates the area of the plane.

        Returns:
            float: The area of the plane.
        """
        A = self._corners[1,:] - self._corners[0,:]
        B = self._corners[2,:] - self._corners[0,:]
        area = np.linalg.norm(np.cross(A, B))
        return area
    
    def get_corners(self):
        return self._corners

    def get_class_id(self): 
        return self._class_id

    def tobytes(self): 
        """
        Converts the plane object to bytes.

        Returns:
            bytes: The plane represented in bytes.
        """
        b = self._corners.astype(np.float32).tobytes() + \
            np.array(self._class_id).astype(np.float32).tobytes()
        return b


class ICPObject:
    """
    A class representing an objec found by the Iterative Closest Point (ICP) algorithm.

    Attributes:
        _transform (ndarray): The transformation matrix of the object.
        _class_id (int): The class ID of the object.
        _rmse (float): The root mean square error (RMSE) from the ICP fit of the object.
        quality (float): Messure in [0,1] of how "good" the detection is 

    Methods:
        __init__(self, transform, class_id, rmse): Initializes the ICPObject instance.
        distance(self, o): Calculates the distance between the object and another object.
        update(self, o): Updates the object with another object's information.
        get_transform(self): Returns the transformation matrix of the object.
        get_class_id(self): Returns the class ID of the object.
        tobytes(self): Converts the object to bytes.
    """


    def __init__(self, transform, class_id, rmse):
        """
        Initializes the ICPObject instance.

        Args:
            transform (ndarray): The transformation matrix of the object.
            class_id (int): The class ID of the object.
            rmse (float): The root mean square error (RMSE) of the object.
        """
        self._transform = transform 
        self._class_id = class_id 
        self._rmse = rmse
        self.quality = 1 - min(1, rmse/RMSE_NORMALIZATION)

    def distance(self, new_object): 
        """
        Calculates the distance between the object and another object.

        If the class IDs of the objects don't match, returns infinity (INF).
        Otherwise, calculates the distance based on the squared difference of transforms.

        Args:
            new_object (ICPObject): The other object to calculate the distance from.

        Returns:
            float: The distance between the objects.
        """
        if self._class_id != new_object.get_class_id():
            return INF 
        sum_squared_diff = np.sum((self._transform - new_object.get_transform()) ** 2)
        return sum_squared_diff

    def update(self, new_object): 
        """
        Updates the object with another object's information.

        Applies a weighted average update to the transformation matrix of the object.

        Args:
            new_object (ICPObject): The object to update from.
        """
        k = ICP_OBJECT_UPDATE_WEIGHT
        self._transform = self._transform * (1-k) + new_object.get_transform() * k 
    
    def get_transform(self): 
        return self._transform

    def get_class_id(self): 
        return self._class_id

    def tobytes(self): 
        """
        Converts the object to bytes.

        Computes the translation and quaternion from the transformation matrix,
        and converts them along with the class ID to bytes.

        Returns:
            bytes: The object represented in bytes.
        """
        translation = self._transform[:3, 3]
        rotation_matrix = self._transform[:3, :3]
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        quaternion = quaternion / np.linalg.norm(quaternion)

        b = translation.astype(np.float32).tobytes() + \
            quaternion.astype(np.float32).tobytes() + \
            np.array([self._class_id]).astype(np.float32).tobytes()
        return b

