import java.net.URI

plugins {
    id("java-library")
    id("org.jetbrains.kotlin.jvm")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

repositories {
    maven { url = URI.create("https://maven.brott.dev/") }
}

dependencies {
    implementation("com.acmerobotics.roadrunner:MeepMeep:0.1.6")
}