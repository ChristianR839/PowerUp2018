/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team839.robot;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 *
 * @author DavidB
 */
public class UniversalJoystick extends Joystick
{
    // JoystickType enum - enum's aren't supported in source -1.3???
    public static final int kUnknown        = 0;
    public static final int kDualAction     = 1;
    public static final int kF310           = 2;
    public static final int kXBox           = 3;
    //public static final int kPS3            = 4;

    public static final int     kBtnA       = 1;
    public static final int     kBtnB       = 2;
    public static final int     kBtnX       = 3;
    public static final int     kBtnY       = 4;
    public static final int     kBtnLB      = 5;
    public static final int     kBtnRB      = 6;
    public static final int     kBtnBack    = 7;
    public static final int     kBtnStart   = 8;
    public static final int     kBtnLStick  = 9;
    public static final int     kBtnRStick  =10;
    public static final int     kBtnLT      =11;
    public static final int     kBtnRT      =12;
    public static final int     kBtnModeA   =13;
    public static final int     kBtnModeB   =14;
    
    
    public static final String  aBtnNames      [] = { "", "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LeftStick", "RightStick", "LeftTrigger", "RightTrigger" };
    public static final int     aXBoxBtns      [] = { 0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10 };
    public static final int     aF310Btns      [] = { 0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10, 11, 12 };
    public static final int     aDualActionBtns[] = { 0, 2, 3, 1, 4, 5, 6, 9, 10, 11, 12 };
    
    public UniversalJoystick( int nJoystick )
    {
        super( nJoystick );
        
//        System.out.print  ( "Joystick Type = ");
//        System.out.println( getJoystickType() );
    }
    
    public JoystickButton createButton( int nBtn )
    {
        return new JoystickButton( this, mapButton(nBtn));
    }
    
    public final int getJoystickType()
    {
    	/**
        DriverStation ds = DriverStation.getInstance();
        
        if (ds.getDigitalIn( 1 ) == true)   return kDualAction;
        if (ds.getDigitalIn( 2 ) == true)   return kF310;        
        if (ds.getDigitalIn( 3 ) == true)   return kXBox;     
        **/   
        
        return kF310;
    }

    public double leftAxisX()
    {
        // Same for all currently support types
        return this.getRawAxis( 0 );
    }
    
    public double leftAxisY()
    {
        // Same for all currently support types
        return this.getRawAxis( 1 );
    }
    
    public double rightAxisX()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
                return this.getRawAxis( 4 );

            case kDualAction:
            default:
                return this.getRawAxis( 3 );
        }
    }

    public double rightAxisY()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
                return this.getRawAxis( 5 );

            case kDualAction:
            default:
                return this.getRawAxis( 4 );
        }
    }
    
    public boolean isTriggerAnalog()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
                return true;
        }
        
        return false;
    }
    
    // returns both left & right triggers as a single -1 to 1 value like an axis
    public double triggers()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
                return this.getRawAxis( 3 );

            case kDualAction:
            default:
            {
                if ( this.getRawButton( 7 ) ) return  1;
                if ( this.getRawButton( 8 ) ) return -1;
                
                return 0;
            }
                
        }
    }
   
    // returns values 0 to 1
    public double leftTrigger()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
            {
                // only return values > 0;
                double dVal = this.getRawAxis( 3 );
                return (dVal < 0) ? 0 : dVal;
            }

            case kDualAction:
            default:
                return (this.getRawButton( 7 ) ? 1 : 0);
        }
    }
    
    // returns values -1 to 0
    public double rightTrigger()
    {
        switch (getJoystickType())
        {
            case kXBox:
            case kF310:
            {
                // only return values < 0;
                double dVal = this.getRawAxis( 3 );
                return (dVal > 0) ? 0 : dVal;
            }

            case kDualAction:
            default:
                return (this.getRawButton( 8 ) ? -1 : 0);
        }
    }
    
    public boolean getBtnA            () { return this.getRawButton( mapButton( 1 )); }
    public boolean getBtnB            () { return this.getRawButton( mapButton( 2 )); }
    public boolean getBtnX            () { return this.getRawButton( mapButton( 3 )); }
    public boolean getBtnY            () { return this.getRawButton( mapButton( 4 )); }
    public boolean getBtnLB           () { return this.getRawButton( mapButton( 5 )); }
    public boolean getBtnRB           () { return this.getRawButton( mapButton( 6 )); }
    public boolean getBtnBack         () { return this.getRawButton( mapButton( 7 )); }
    public boolean getBtnStart        () { return this.getRawButton( mapButton( 8 )); }
    public boolean getBtnLeftStick    () { return this.getRawButton( mapButton( 9 )); }
    public boolean getBtnRightStick   () { return this.getRawButton( mapButton(10 )); }
    public boolean getBtnLeftTrigger  () { return this.getRawButton( mapButton(11 )); }
    public boolean getBtnRightTrigger () { return this.getRawButton( mapButton(12 )); }
            
    public int mapButton( int nBtn )
    {
        int[] aMap = null;
        
        switch (getJoystickType())
        {
            case kXBox      : aMap = aXBoxBtns;         break;
            case kF310      : aMap = aF310Btns;         break;
            case kDualAction: aMap = aDualActionBtns;   break;
        }
        
        if (aMap != null)
        {
            // -=>TODO: should check to see if nBtn is valid
            return aMap[ nBtn ];
        }
        
        return nBtn;
    }
    
}
