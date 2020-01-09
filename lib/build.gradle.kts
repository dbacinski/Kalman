plugins {
    kotlin("jvm") version "1.3.61"
}

dependencies {
    implementation(kotlin("stdlib"))
    implementation("com.googlecode.efficient-java-matrix-library:ejml:0.25")
    testImplementation("org.junit.jupiter:junit-jupiter-api:5.5.2")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine:5.5.2")
    testImplementation("org.junit.jupiter:junit-jupiter-params:5.5.2")
    testImplementation("io.mockk:mockk:1.9.3")
    testImplementation("ch.tutteli.atrium:atrium-fluent-en_GB:0.9.0-alpha")
    testImplementation("io.jenetics:jpx:1.6.0")
}



