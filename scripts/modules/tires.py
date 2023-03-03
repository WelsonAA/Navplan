#!/usr/bin/env python3
import numpy as np

class Tire:
    def characteristic(self,alpha,*args):
        pass
    
class TireLinear(Tire):
    def __init__(self,k=40000):
        self.k = k
        
    def characteristic(self,alpha,*args):
        Fy = -self.k * alpha
        return Fy
    
class TirePolynomial(Tire):
    def __init__(self,k1=115000,k2=560000):
        self.k1 = k1
        self.k2 = k2
    
    def characteristic(self,alpha,*args):
        Fy = - (self.k1 * alpha - self.k2 * alpha**3)
        return Fy
    
class TirePacejka(Tire):
    def __init__(self):
        self.a0 = 1    # Shape factor
        self.a1 = 0    # Load dependency of lateral friction (*1000) [1/kN] 
        self.a2 = 800  # Lateral friction level (*1000) [-]
        self.a3 = 3000 # Maximum cornering stiffness [N/deg]
        self.a4 = 50   # Load at maximum cornering stiffness [kN]
        self.a5 = 0    # Camber sensitivity of cornering stiffness
        self.a6 = 0    # Load dependency of curvature factor
        self.a7 = -1   # Curvature factor level
        self.a8 = 0    # Camber sensitivity of horizontal shift
        self.a9 = 0    # Load dependency of horizontal shift
        self.a10 = 0   # Horizontal shift level
        self.a11 = 0   # Combined load and camber sensitivity of vertical shift
        self.a12 = 0   # Load dependency of vertical shift
        self.a13 = 0   # Vertical shift level
        
    def characteristic(self,alpha,*args):
        '''
        Args:
            alpha: slip angle (rad)
            Fz (args[0]): load (Newton)
            muy (args[1]): Lateral friction coefficient (*1000)
        
        Returns:
            Fy: Lateral Force
        '''
        Fz = args[0]
        muy = args[1]
        
        # Slip angle treatment
        ALPHA = np.arcsin(np.sin(alpha)) # [rad]
        ALPHA = 180 / np.pi * ALPHA    # Conversion [rad] to [deg]
        
        # Parameters
        a0 = self.a0
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        a5 = self.a5
        a6 = self.a6
        a7 = self.a7
        a8 = self.a8
        a9 = self.a9
        a10 = self.a10
        a11 = self.a11
        a12 = self.a12
        a13 = self.a13

        Fz = Fz/1000 # Conversion [N] - [kN]

        camber = 0 # Camber angle

        C = a0 # Shape factor
        muy0 = a1 * Fz + a2 # Lateral friction coefficient nominal [-]
        muy = muy * 1000 # Lateral friction coefficient operational
        D = muy0 * Fz # muy = lateral friction coefficient
        BCD = a3 * np.sin(2 * np.arctan(Fz/a4))*(1-a5 * abs(camber)) # Cornering stiffness
        E = a6 * Fz + a7 # Curvature factor
        B = BCD/(C * D) # stiffness factor
        Sh = a8 * camber + a9 * Fz + a10 # Horizontal shift
        Sv = a11 * Fz * camber + a12 * Fz + a13 # Vertical shift
        ALPHAeq = muy0/muy*(ALPHA + Sh) # Equivalent slip angle
         
        # Reference characteristics
        fy = D * np.sin(C * np.arctan(B * ALPHAeq - E*(B * ALPHAeq - np.arctan(B * ALPHAeq))))
         
        # Lateral force
        Fy = -muy/muy0*(fy + Sv)
        
        return Fy