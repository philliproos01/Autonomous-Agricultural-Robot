<?php
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    $latitudes = array();
    $longitudes = array();

    for ($i = 1; $i <= 5; $i++) {
        // if matches the form: "Lat#d: [lat as double]", "Long#d: [long as double]"
        if (preg_match('/^Lat#\d: (-?\d+(\.\d+))$/', $_POST['latitude' . $i], $latmatches) && preg_match('/^Long#\d: (-?\d+(\.\d+))$/', $_POST['longitude' . $i], $longmatches)) {
            $latitudes[] = $latmatches[1];
            $longitudes[] = $longmatches[1];
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