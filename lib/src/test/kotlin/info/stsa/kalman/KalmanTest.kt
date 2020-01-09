package info.stsa.kalman

import ch.tutteli.atrium.api.fluent.en_GB.toBe
import ch.tutteli.atrium.api.verbs.expect
import io.jenetics.jpx.GPX
import io.jenetics.jpx.Track
import io.jenetics.jpx.TrackSegment
import io.jenetics.jpx.WayPoint
import org.junit.jupiter.api.Test


class KalmanTest {

    @Test
    fun `smooth`() {
        val kalman = Kalman1D()

        kalman.init(1.0)
        val input = listOf(
            1.0,
            10.0,
            2.0,
            3.0
        )
        val result = kalman.smooth(input)
        expect(result).toBe(
            listOf(
                0.9999991254068846,
                1.8561235063547405,
                1.8698097465864196,
                1.9773190062564472
            )
        )
    }

    @Test
    fun `smooth pairs`() {
        val kalman = Kalman2D()
        kalman.init(1.0 to 1.0)
        val input = listOf(
            1.0 to 1.0,
            10.0 to 10.0,
            2.0 to 2.0,
            3.0 to 3.0
        )
        val result = kalman.smooth(input)
        expect(result).toBe(
            listOf(
                1.0000001702228285 to 1.0000001702228285,
                10.543148596832522 to 10.543148596832522,
                6.572048404662247 to 6.572048404662247,
                5.458222906721857 to 5.458222906721857
            )
        )
    }

    @Test
    fun `smooth triple`() {
        val kalman = Kalman3D()
        kalman.init(Triple(1.0, 1.0, 1.0))

        val input =
            listOf(
                Triple(1.0, 1.0, 1.0),
                Triple(10.0, 10.0, 10.0),
                Triple(2.0, 2.0, 2.0),
                Triple(3.0, 3.0, 3.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(10.0, 10.0, 10.0),
                Triple(3.0, 3.0, 3.0),
                Triple(2.0, 2.0, 2.0),
                Triple(1.0, 1.0, 1.0)
            )
        val result = kalman.smooth(input)
//        expect(result).toBe(
//            listOf(
//                Triple(0.9999991254068846, 0.9999991254068846, 0.9999991254068846),
//                Triple(11.613956640805378, 11.613956640805378, 11.613956640805378),
//                Triple(6.533468400176047, 6.533468400176047, 6.533468400176047),
//                Triple(5.271282958648654, 5.271282958648654, 5.271282958648654)
//            )
//        )
        print(result.joinToString(separator = "\n"))
    }

    @Test
    fun `smooth gps location`() {
        val kalman = GeoKalman()
        val inputLocations =
            listOf(
                GpsLocation(1578419833440, 55.2897392, 22.8055557, 27.0),
                GpsLocation(1578419843440, 55.2897392, 22.8055557, 27.0),
                GpsLocation(1578419853440, 55.2897392, 22.8055557, 27.0),
                GpsLocation(1578419863440, 55.2899307, 22.8049623, 103.0),
                GpsLocation(1578419879181, 55.289884, 22.805904, 16.0),
                GpsLocation(1578419934442, 55.2900883, 22.79782, 136.0),
                GpsLocation(1578419967000, 55.28977, 22.8052417, 10.0),
                GpsLocation(1578420027000, 55.2911567, 22.79295, 13.0),
                GpsLocation(1578420070000, 55.2899367, 22.8062333, 12.0),
                GpsLocation(1578420151492, 55.2897392, 22.8055557, 27.0),
                GpsLocation(1578420162492, 55.2897392, 22.8055557, 27.0),
                GpsLocation(1578420173492, 55.2897392, 22.8055557, 27.0)

            )
        val input = inputLocations.map { it.toTriple() }

//        kalman.init(input.first())

        val result = kalman.smooth(input)

        val resultLocations = result.map { it.toGpsLocation() }
        print(resultLocations)

        storeToFile(input, "input.gpx")
        storeToFile(result, "output.gpx")
    }

    private fun storeToFile(
        input: List<Triple<Double, Double, Long>>,
        fileName: String
    ) {
        val gpx = GPX.builder()
            .addTrack { track: Track.Builder ->
                track
                    .addSegment { segment: TrackSegment.Builder ->
                        segment.apply {
                            input.forEach {
                                addPoint { p: WayPoint.Builder ->
                                    p.lat(it.first).lon(it.second).time(it.third)
                                }
                            }
                        }
                    }
            }
            .build()
        GPX.write(gpx, fileName)
    }
}

data class GpsLocation(val time: Long, val lat: Double, val lng: Double, val accuracy: Double) {
    override fun toString(): String {
        return "$time $lat $lng acc=${accuracy.toInt()}\n"
    }
}

fun GpsLocation.toTriple(): Triple<Double, Double, Long> =
    Triple(lat, lng, time)

fun Triple<Double, Double, Long>.toGpsLocation() =
    GpsLocation(third, first, second, 0.0)



