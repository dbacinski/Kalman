package info.stsa.kalman

import ikalman.GeoTrackFilter
import ikalman.GpsKalmanFilter
import jama.Matrix
import jkalman.JKalman
import kotlin.math.abs
import kotlin.math.max


class Kalman1D {

    private val kalman = JKalman(2, 1)

    /**
     * @param init start with given value
     */
    fun init(init: Double = 0.0, precision: Double = 0.000001) {
        var predict = predict(init)
        var i = 0
        while (delta(init, predict) > precision
        ) {
            if (i > 1000) break
            predict = predict(init)
            i++
        }

        println("Warmed up after i=$i")
    }

    /**
     * Smoothens Double value stream using Kalman filter.
     *
     */
    fun smooth(input: List<Double>): List<Double> {


        // transitions for x, dx
        val tr = arrayOf(
            arrayOf(1.0, 0.0).toDoubleArray(),
            arrayOf(0.0, 1.0).toDoubleArray()
        )
        kalman.transition_matrix = Matrix(tr)
        kalman.error_cov_post = kalman.error_cov_post.identity()

        return input.map { value ->
            predict(value)
        }
    }

    private fun predict(value: Double): Double {
        val m = Matrix(1, 1)
        m.set(0, 0, value)
        // corrected state [x, dx]
        val c = kalman.Correct(m)
        // state [x, dx]
        kalman.Predict()
        return c.get(0, 0)
    }
}

class Kalman2D {

    private val kalman = JKalman(4, 2)

    /**
     * @param init start with given value
     */
    fun init(init: Pair<Double, Double> = Pair(0.0, 0.0), precision: Double = 0.000001) {
        var predict = predictPair(init)
        var i = 0
        while (delta(init.first, predict.first) > precision
            || delta(init.second, predict.second) > precision
        ) {
            if (i > 1000) break
            predict = predictPair(init)
            i++
        }

        println("Warmed up after i=$i")
    }

    /**
     * Smoothens Pair<Double, Double> values stream using Kalman filter.
     */
    fun smooth(input: List<Pair<Double, Double>>, dx: Double = 1.0, dy: Double = 1.0): List<Pair<Double, Double>> {


        // transitions for x, y, dx, dy
        val tr = arrayOf(
            arrayOf(1.0, 0.0, dx, 0.0).toDoubleArray(),
            arrayOf(0.0, 1.0, 0.0, dy).toDoubleArray(),
            arrayOf(0.0, 0.0, 1.0, 0.0).toDoubleArray(),
            arrayOf(0.0, 0.0, 0.0, 1.0).toDoubleArray()
        )

        kalman.transition_matrix = Matrix(tr)
        kalman.error_cov_post = kalman.error_cov_post.identity()


        return input.map { value ->
            predictPair(value)
        }
    }

    private fun predictPair(value: Pair<Double, Double>): Pair<Double, Double> {
        val m = Matrix(2, 1)
        m.set(0, 0, value.first)
        m.set(1, 0, value.second)

        // corrected state [x, dx]
        val correct = kalman.Correct(m)
        val corrected = correct.get(0, 0) to correct.get(1, 0)

        // state [x, dx]
        val predict = kalman.Predict()
        val predicted = predict.get(0, 0) to predict.get(1, 0)
//        println("value=$value predicted=$predicted corrected=$corrected")
        return predicted
    }

}

class GeoKalman {

    private val kalman = GpsKalmanFilter(1.0)
    private var lastTimestamp: Long? = null

    /**
     * @param init start with given value
     */
    fun init(init: Triple<Double, Double, Long> = Triple(0.0, 0.0, 0L), precision: Double = 0.000001) {
        var predict = predictPair(init)
        var i = 0
        while (delta(init.first, predict.first) > precision
            || delta(init.second, predict.second) > precision
        ) {
            if (i > 1000) break
            predict = predictPair(init)
            i++
        }

        println("Warmed up after i=$i")
    }

    /**
     * Smoothens Pair<Double, Double> values stream using Kalman filter.
     */
    fun smooth(
        input: List<Triple<Double, Double, Long>>
    ): List<Triple<Double, Double, Long>> {

        return input.map { value ->
            predictPair(value)
        }
    }

    private fun predictPair(value: Triple<Double, Double, Long>): Triple<Double, Double, Long> {
        val timestamp = value.third
        val secondsPassed = if (lastTimestamp == null) {
            lastTimestamp = timestamp
            0.0
        } else {
            val delta = (timestamp - lastTimestamp!!) / 1000.0
            lastTimestamp = timestamp
            delta
        }
        kalman.updateVelocity2d(value.first, value.second, secondsPassed)
        kalman.position
        val latLng = kalman.position
        val velocity = kalman.velocity
        val speed = kalman.kmh
        val bearing = kalman.bearing
        println("time:$timestamp deltaTime:${secondsPassed} position:$latLng v:${velocity.joinToString()} speed:$speed bear:$bearing ")

        return Triple(latLng.lat, latLng.lng, timestamp)
    }
}

class Kalman3D {

    val kalman = JKalman(6, 3)

    /**
     * @param init start with given value
     */
    fun init(init: Triple<Double, Double, Double> = Triple(0.0, 0.0, 0.0), precision: Double = 0.000001) {
        var predict = measureTriple(init)
        var i = 0
        while (delta(init.first, predict.first) > precision
            || delta(init.second, predict.second) > precision
            || delta(init.third, predict.third) > precision
        ) {
            if (i > 1000) break

            measureTriple(init)
            predict = predictTriple()
            i++
        }

        println("Warmed up after i=$i")
    }


    /**
     * Smoothens Triple<Double, Double, Double> values list using Kalman filter.
     */
    fun smooth(
        input: List<Triple<Double, Double, Double>>,
        dx: Double = 0.1,
        dy: Double = 1.0,
        dz: Double = 10.0
    ): List<Triple<Double, Double, Double>> {

        // transitions for x, dx
        val tr = arrayOf(
            arrayOf(1.0, 0.0, 0.0, dx, 0.0, 0.0).toDoubleArray(),
            arrayOf(0.0, 1.0, 0.0, 0.0, dy, 0.0).toDoubleArray(),
            arrayOf(0.0, 0.0, 1.0, 0.0, 0.0, dz).toDoubleArray(),
            arrayOf(0.0, 0.0, 0.0, 1.0, 0.0, 0.0).toDoubleArray(),
            arrayOf(0.0, 0.0, 0.0, 0.0, 1.0, 0.0).toDoubleArray(),
            arrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 1.0).toDoubleArray()
        )
        kalman.transition_matrix = Matrix(tr)
        kalman.error_cov_post = kalman.error_cov_post.identity()
        return input.map { value ->
            predictTriple()
            measureTriple(value)
        }
    }

    private fun measureTriple(
        value: Triple<Double, Double, Double>
    ): Triple<Double, Double, Double> {
        // measurement [x]
        val m = Matrix(3, 1)
        m.set(0, 0, value.first)
        m.set(1, 0, value.second)
        m.set(2, 0, value.third)

        // corrected state [x, y, z, dx, dy, dz]
        val correct = kalman.Correct(m)
        return Triple(correct.get(0, 0), correct.get(1, 0), correct.get(2, 0))
    }

    private fun predictTriple(): Triple<Double, Double, Double> {
        // state [x, y, z, dx, dy, dz]
        val predict = kalman.Predict()
        return Triple(predict.get(0, 0), predict.get(1, 0), predict.get(2, 0))
    }
}

private fun delta(d1: Double, d2: Double): Double {
    return abs(d1 - d2) / max(abs(d1), abs(d2))
}
