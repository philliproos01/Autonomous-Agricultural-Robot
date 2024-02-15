<?php
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    $latitudes = array();
    $longitudes = array();

    for ($i = 1; $i <= 5; $i++) {
        if (preg_match('/^Lat#/', $_POST['latitude' . $i]) && preg_match('/^Long#/', $_POST['longitude' . $i])) {
            $latitudes[] = $_POST['latitude' . $i];
            $longitudes[] = $_POST['longitude' . $i];
        }
    }

    $file = fopen("test/robot.txt", "w");
    $coordinates = "";
    for ($i = 0; $i < count($latitudes); $i++) {
        $coordinates .= $latitudes[$i] . ", " . $longitudes[$i] . "\n";
    }
    fwrite($file, $coordinates);
    fclose($file);
    header("Location: index.html");
    exit;
}
?>