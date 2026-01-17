package org.firstinspires.ftc.teamcode.misc

enum class ArtifactColors {
    PURPLE,
    GREEN,
    NONE,
}

enum class Motif(val first: ArtifactColors, val second: ArtifactColors, val third: ArtifactColors) {
    PURPLE_PURPLE_GREEN(ArtifactColors.PURPLE, ArtifactColors.PURPLE, ArtifactColors.GREEN),
    PURPLE_GREEN_PURPLE(ArtifactColors.PURPLE, ArtifactColors.GREEN, ArtifactColors.PURPLE),
    GREEN_PURPLE_PURPLE(ArtifactColors.GREEN, ArtifactColors.PURPLE, ArtifactColors.PURPLE),
}
