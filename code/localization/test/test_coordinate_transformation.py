import importlib

import pytest

pyproj = pytest.importorskip("pyproj")

coordinate_transformation = importlib.import_module(
    "localization.coordinate_transformation"
)
CoordinateTransformer = coordinate_transformation.CoordinateTransformer
extract_geo_reference_from_opendrive = (
    coordinate_transformation.extract_geo_reference_from_opendrive
)


pytestmark = pytest.mark.unit


OPEN_DRIVE = """
<OpenDRIVE>
  <header>
        <geoReference><![CDATA[
            +proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0
            +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs
        ]]></geoReference>
  </header>
</OpenDRIVE>
"""


OPEN_DRIVE_NON_DEFAULT = """
<OpenDRIVE>
    <header>
                <geoReference><![CDATA[
                        +proj=tmerc +lat_0=1 +lon_0=2 +k=0.9996 +x_0=500000 +y_0=1000000
                        +datum=WGS84 +units=m
                        +geoidgrids=egm96_15.gtx +vunits=m +no_defs
                ]]></geoReference>
    </header>
</OpenDRIVE>
"""


@pytest.fixture(autouse=True)
def reset_transformer_state():
    CoordinateTransformer.la_ref = 0.0
    CoordinateTransformer.ln_ref = 0.0
    CoordinateTransformer.h_ref = 0.0
    CoordinateTransformer.geo_reference = None
    CoordinateTransformer._geodetic_to_local = None
    CoordinateTransformer._use_projected_transform = False
    CoordinateTransformer.ref_set = False


def test_extract_geo_reference_from_opendrive():
    assert " ".join(extract_geo_reference_from_opendrive(OPEN_DRIVE).split()) == (
        "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 "
        "+units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs"
    )


def test_configure_from_geo_reference_sets_projection_and_reference():
    CoordinateTransformer.configure_from_geo_reference(
        extract_geo_reference_from_opendrive(OPEN_DRIVE)
    )

    assert CoordinateTransformer.ref_set is True
    assert CoordinateTransformer.la_ref == pytest.approx(0.0)
    assert CoordinateTransformer.ln_ref == pytest.approx(0.0)
    assert CoordinateTransformer._geodetic_to_local is not None


def test_gnss_to_xyz_uses_legacy_xy_for_default_carla_projection():
    geo_reference = extract_geo_reference_from_opendrive(OPEN_DRIVE)
    CoordinateTransformer.configure_from_geo_reference(geo_reference)

    transformer = CoordinateTransformer()
    expected_x, expected_y, _ = transformer.geodetic_to_enu(0.0015, 0.001, 12.0)

    x, y, z = transformer.gnss_to_xyz(0.0015, 0.001, 12.0)

    assert x == pytest.approx(expected_x)
    assert y == pytest.approx(expected_y)
    assert z == pytest.approx(12.0)


def test_gnss_to_xyz_uses_opendrive_projection_for_non_default_reference():
    geo_reference = extract_geo_reference_from_opendrive(OPEN_DRIVE_NON_DEFAULT)
    CoordinateTransformer.configure_from_geo_reference(geo_reference)

    sanitized_geo_reference = (
        "+proj=tmerc +lat_0=1 +lon_0=2 +k=0.9996 +x_0=500000 +y_0=1000000 +datum=WGS84 "
        "+units=m +no_defs"
    )
    projection = pyproj.Transformer.from_crs(
        pyproj.CRS.from_epsg(4326),
        pyproj.CRS.from_user_input(sanitized_geo_reference),
        always_xy=True,
    )
    expected_x, expected_y = projection.transform(2.001, 1.0015)

    transformer = CoordinateTransformer()
    x, y, z = transformer.gnss_to_xyz(1.0015, 2.001, 12.0)

    assert x == pytest.approx(expected_x)
    assert y == pytest.approx(-expected_y)
    assert z == pytest.approx(12.0)
