package org.wildstang.sample.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.sample.auto.program.FeedFullField;
import org.wildstang.sample.auto.program.FullFieldLeft;
import org.wildstang.sample.auto.program.FullFieldRight;
import org.wildstang.sample.auto.program.HalfFieldLeft;
import org.wildstang.sample.auto.program.Trench;
import org.wildstang.sample.auto.program.ShootMid;


/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    //SAMPLE_PROGRAM("Sample", SampleAutoProgram.class),
    FULL_FEED_LEFT("FullFieldLeft", FullFieldLeft.class),
    FEED_FULL_FIELD("FeedFullField", FeedFullField.class),
    FULL_FEED_RIGHT("FullFieldRight", FullFieldRight.class),
    HALF_FIELD_LEFT("HalfFieldLeft", HalfFieldLeft.class),
    TRENCH("Trench", Trench.class),
    SHOOT_MID("ShootMid", ShootMid.class)
    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> programClass;

    /**
     * Initialize name and AutoProgram map.
     * @param name Name, must match that in class to prevent errors.
     * @param programClass Class containing AutoProgram
     */
    WsAutoPrograms(String name, Class<?> programClass) {
        this.name = name;
        this.programClass = programClass;
    }

    /**
     * Returns the name mapped to the AutoProgram.
     * @return Name mapped to the AutoProgram.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns AutoProgram's class.
     * @return AutoProgram's class.
     */
    @Override
    public Class<?> getProgramClass() {
        return programClass;
    }
}